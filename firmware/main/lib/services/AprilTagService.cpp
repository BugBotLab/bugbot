// ARUCO_DB must be defined before ArucoLite.h — selects the marker database.
// APRILTAG_16h5 keeps the same family as before so all existing printed markers work.
#define ARUCO_DB ARUCO_DB_APRILTAG_16h5
#include "AprilTagService.h"
#include <ArucoLite.h>
#include <math.h>

// Detector for QQVGA (160×120), up to 8 markers per frame.
// Frame buffer (19 200 B, GRAYSCALE) is zero-copied directly from the camera fb.
using Aruco_t = ArucoLite<160, 120, 8>;

// EMA constants — tuned for QQVGA AprilTag geometry.
// Az and dist derived from ~100 px tag sides → α=0.35 (≈3 frame TC) adequate.
// Yaw derived from horizontal apparent width that shrinks to ~15 px at 79° oblique;
// ±1 px corner error → ±7% → ±8° yaw noise, so α=0.12 (≈8 frame TC).
static constexpr float    kAlphaAz   = 0.35f;
static constexpr float    kAlphaDist = 0.80f;
static constexpr float    kAlphaYaw  = 0.12f;
static constexpr float    kYawGate   = 45.0f;  // rejects ±22→∓22 sign-flip artefacts
static constexpr uint32_t kEmaResetMs = 2000;  // 30 fps → allows 60 missed frames

// ── geometry helpers ──────────────────────────────────────────────────────────

static inline float ptDist_(const pt2d_t& a, const pt2d_t& b) {
    const float dx = b.x - a.x, dy = b.y - a.y;
    return sqrtf(dx * dx + dy * dy);
}

// ArucoLite normalises corners to: pt[0]=TL, pt[1]=TR, pt[2]=BR, pt[3]=BL.
static void cornersToHit_(const aruco_t& det,
                          float f_px, float cam_cx, float cam_cy,
                          float tag_size_mm,
                          WebWS::AprilTagHit& out)
{
    const pt2d_t& tl = det.pt[0];
    const pt2d_t& tr = det.pt[1];
    const pt2d_t& br = det.pt[2];
    const pt2d_t& bl = det.pt[3];

    out.cx_px = (tl.x + tr.x + br.x + bl.x) * 0.25f;
    out.cy_px = (tl.y + tr.y + br.y + bl.y) * 0.25f;

    // hmirror=1 inverts the image-x / camera-frame-x relationship.
    out.az_deg = atan2f(out.cx_px - cam_cx, f_px) * (180.0f / M_PI);
    out.el_deg = atan2f(-(out.cy_px - cam_cy), f_px) * (180.0f / M_PI);

    const float top   = ptDist_(tl, tr);
    const float right = ptDist_(tr, br);
    const float bot   = ptDist_(br, bl);
    const float left  = ptDist_(bl, tl);

    // Geometric mean of the two vertical sides for stable distance at any yaw.
    const float geo_vert = (left > 0.5f && right > 0.5f) ? sqrtf(left * right) : 0.0f;
    out.dist_mm = (geo_vert > 0.5f) ? (f_px * tag_size_mm / geo_vert) : 9999.0f;

    // Tag yaw via horizontal foreshortening: cos(θ) = avg_horiz / avg_vert.
    const float avg_horiz = (top + bot) * 0.5f;
    const float avg_vert  = (left + right) * 0.5f;
    const float cos_yaw   = (avg_vert > 1.0f) ? fminf(1.0f, avg_horiz / avg_vert) : 1.0f;
    const float sign      = (right >= left) ? +1.0f : -1.0f;
    out.tag_yaw_deg = sign * acosf(cos_yaw) * (180.0f / M_PI);
}

// ── ICVConsumer lifecycle ─────────────────────────────────────────────────────

bool AprilTagService::begin(WebWS& ws, float tag_size_mm, float fov_h_deg) {
    ws_        = &ws;
    tagSizeMm_ = tag_size_mm;
    fovH_deg_  = fov_h_deg;
    // Pre-compute intrinsics for QQVGA 160×120.
    camCx_ = 80.0f;
    camCy_ = 60.0f;
    fPx_   = camCx_ / tanf(fov_h_deg * 0.5f * M_PI / 180.0f);
    return true;
}

void AprilTagService::onActivate() {
    if (!arucoMem_) {
        arucoMem_ = heap_caps_malloc(sizeof(Aruco_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (arucoMem_) {
            new (arucoMem_) Aruco_t();
            Serial.println("[ArUco] detector allocated in PSRAM");
        } else {
            Serial.println("[ArUco] PSRAM alloc failed");
        }
    }
    memset(smooth_, 0, sizeof(smooth_));
    seq_ = 0;
    lastFpsMs_ = 0;
    totalFrames_ = 0;
    foundFrames_ = 0;
}

void AprilTagService::onDeactivate() {
    if (arucoMem_) {
        reinterpret_cast<Aruco_t*>(arucoMem_)->~Aruco_t();
        heap_caps_free(arucoMem_);
        arucoMem_ = nullptr;
        Serial.println("[ArUco] detector freed");
    }
}

// ── per-frame detection ───────────────────────────────────────────────────────

void AprilTagService::onFrame(camera_fb_t* fb) {
    if (!arucoMem_) return;
    Aruco_t* aruco = reinterpret_cast<Aruco_t*>(arucoMem_);

    const uint32_t frameMs = (uint32_t)millis();
    const uint32_t t1 = micros();
    Aruco_t::Frame* frame = reinterpret_cast<Aruco_t::Frame*>(fb->buf);
    aruco->process(*frame);
    const uint32_t t2 = micros();

    totalFrames_++;
    if (aruco->arucos_found > 0) foundFrames_++;
    if (frameMs - lastFpsMs_ >= 1000) {
        const uint32_t elapsed = frameMs - lastFpsMs_;
        Serial.printf("[ArUco] %.1f fps  detect=%.1f fps  process=%lu us\n",
                      totalFrames_ * 1000.0f / elapsed,
                      foundFrames_ * 1000.0f / elapsed,
                      (unsigned long)(t2 - t1));
        lastFpsMs_   = frameMs;
        totalFrames_ = 0;
        foundFrames_ = 0;
    }

    const int n = aruco->arucos_found;

    if (n > 0 && ws_) {
        WebWS::AprilTagHit hits[4];
        int nGood = 0;
        const int check_n = (n > 8) ? 8 : n;

        for (int i = 0; i < check_n && nGood < 4; i++) {
            const aruco_t& det = aruco->result[i];

            // Require all 4 sides to be at least 3 px — filters sub-pixel noise hits.
            bool sane = true;
            for (int e = 0; e < 4 && sane; e++) {
                const pt2d_t& a = det.pt[e];
                const pt2d_t& b = det.pt[(e + 1) & 3];
                const float dx = b.x - a.x, dy = b.y - a.y;
                if (dx * dx + dy * dy < 9.0f) sane = false;
            }
            if (!sane) continue;

            hits[nGood].id = (uint8_t)det.aruco_idx;
            cornersToHit_(det, fPx_, camCx_, camCy_, tagSizeMm_, hits[nGood]);

            // Per-tag EMA smoothing
            const uint8_t tid = hits[nGood].id;
            if (tid < 16) {
                TagSmooth& s = smooth_[tid];
                if (!s.valid || (frameMs - s.lastSeenMs) > kEmaResetMs) {
                    s.az    = hits[nGood].az_deg;
                    s.dist  = hits[nGood].dist_mm;
                    s.yaw   = hits[nGood].tag_yaw_deg;
                    s.valid = true;
                } else {
                    s.az   += kAlphaAz   * (hits[nGood].az_deg  - s.az);
                    s.dist += kAlphaDist * (hits[nGood].dist_mm - s.dist);
                    const float yaw_innov = hits[nGood].tag_yaw_deg - s.yaw;
                    if (fabsf(yaw_innov) < kYawGate) {
                        s.yaw += kAlphaYaw * yaw_innov;
                    }
                }
                s.lastSeenMs = frameMs;
                hits[nGood].az_deg      = s.az;
                hits[nGood].dist_mm     = s.dist;
                hits[nGood].tag_yaw_deg = s.yaw;
            }
            nGood++;
        }

        if (nGood > 0) {
            if (tagCallback_) tagCallback_((uint8_t)nGood, hits);
            // Rate-limit WS sends to 5 fps — flooding at 10+ fps causes client drops.
            static uint32_t lastWsSendMs = 0;
            if (frameMs - lastWsSendMs >= 200) {
                ws_->sendAprilTags(seq_, frameMs, (uint8_t)nGood, hits);
                lastWsSendMs = frameMs;
            }
            if (frameMs - lastSnapshotMs_ >= 2000) {
                lastSnapshotMs_ = frameMs;
            }
            seq_++;
        } else {
            if (tagCallback_) tagCallback_(0, nullptr);
        }
    } else {
        if (tagCallback_) tagCallback_(0, nullptr);
    }
}
