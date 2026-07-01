#include "PoseService.h"
#include "MotionService.h"
#include "OdomService.h"
#include <math.h>

bool PoseService::begin(PoseBus& bus, MotionService& motion, const KinematicModel& model,
                        LidarService* lidar, OdomService* odom) {
  bus_    = &bus;
  motion_ = &motion;
  model_  = &model;
  lidar_  = lidar;
  odom_   = odom;
  // 4 KB stack, priority 2, Core 1
  return CreateTaskPinnedPSRAM("pose", &PoseService::taskThunk, 4, 2, &th_, 1, this);
}

void PoseService::onAprilTags(uint8_t n, const WebWS::AprilTagHit* hits) {
  if (n == 0) return;
  portENTER_CRITICAL(&tagMux_);
  nLatestTags_ = 0;
  for (uint8_t i = 0; i < n && nLatestTags_ < 4; i++) {
    const auto& h = hits[i];
    if (h.id >= 16 || h.dist_mm < 50.0f) continue;
    latestTags_[nLatestTags_++] = { h.id, h.az_deg, h.dist_mm, h.tag_yaw_deg,
                                    (uint32_t)millis(), true };
  }
  portEXIT_CRITICAL(&tagMux_);
}

void PoseService::startLocalizationSweep() {
  if (locSweepActive_) return;
  sweepAccumYaw_ = 0.0f;
  sweepLastYaw_  = yaw_;
  sweepTagSeen_  = false;
  locSweepActive_ = true;
  Serial.println("[Pose] Localization sweep started");
}

void PoseService::stopLocalizationSweep() {
  if (!locSweepActive_) return;
  locSweepActive_ = false;
  if (motion_) motion_->setCommandVec(0.0f, 0.0f, 0.0f);
}

void PoseService::resetPose() {
  x_ = 0.0f; y_ = 0.0f; yaw_ = 0.0f;
  p00_ = 400.0f; p01_ = 0.0f; p02_ = 0.0f;
  p11_ = 400.0f; p12_ = 0.0f;
  p22_ = 1.0f;
}

void PoseService::resetMap() {
  portENTER_CRITICAL(&mapMux_);
  for (auto& t : tagMap_) t.known = false;
  nKnownTags_ = 0;
  portEXIT_CRITICAL(&mapMux_);
}

void PoseService::defineTag(uint8_t id, float x_mm, float y_mm, float face_deg) {
  if (id >= 16) return;
  const float face_rad = face_deg * ((float)M_PI / 180.0f);
  portENTER_CRITICAL(&mapMux_);
  const bool wasKnown = tagMap_[id].known;
  tagMap_[id] = { x_mm, y_mm, face_rad, true };
  if (!wasKnown) nKnownTags_++;
  portEXIT_CRITICAL(&mapMux_);
  Serial.printf("[Pose] tag %u defined at world (%.0f, %.0f) face=%.1f deg\n",
                id, x_mm, y_mm, face_deg);
}

void PoseService::applyArenaLayout(float W, float H, float /*c*/) {
  arenaW_ = W;
  arenaH_ = H;
  resetMap();
  // 2 tags per wall at 1/3 and 2/3, clockwise from top-left
  defineTag(0,  W/3,     H,       270.0f); // top wall    → facing down
  defineTag(1,  2*W/3,   H,       270.0f); // top wall    → facing down
  defineTag(2,  W,       2*H/3,   180.0f); // right wall  → facing left
  defineTag(3,  W,       H/3,     180.0f); // right wall  → facing left
  defineTag(4,  2*W/3,   0.0f,    90.0f);  // bottom wall → facing up
  defineTag(5,  W/3,     0.0f,    90.0f);  // bottom wall → facing up
  defineTag(6,  0.0f,    H/3,     0.0f);   // left wall   → facing right
  defineTag(7,  0.0f,    2*H/3,   0.0f);   // left wall   → facing right
  Serial.printf("[Arena] layout applied W=%.0f H=%.0f (2-per-wall thirds)\n", W, H);
}

// ── EKF predict ───────────────────────────────────────────────────────────────
// Cross-coupling terms from the rotation Jacobian:
//   dx/dtheta = -dy_w,  dy/dtheta = dx_w
// These grow p02 and p12 during motion so that subsequent tag yaw corrections
// also tighten the position estimate (and vice-versa).
void PoseService::predict_(float dx_w, float dy_w, float dyaw,
                           float q_pos, float q_yaw) {
  x_   += dx_w;
  y_   += dy_w;
  yaw_ += dyaw;
  while (yaw_ >  (float)M_PI) yaw_ -= 2.0f * (float)M_PI;
  while (yaw_ < -(float)M_PI) yaw_ += 2.0f * (float)M_PI;

  p00_ += q_pos;
  p11_ += q_pos;
  p22_ += q_yaw;
  // First-order cross terms (second-order negligible at 50 Hz / small steps)
  p01_ += dx_w * p02_ - dy_w * p12_ - dy_w * dx_w * p22_;
  p02_ += -dy_w * p22_;
  p12_ +=  dx_w * p22_;
}

// ── EKF scalar measurement update ─────────────────────────────────────────────
// Applies one scalar measurement z = h0*x + h1*y + h2*yaw + noise(R).
// Caller computes and angle-wraps the innovation before calling.
void PoseService::updateScalar_(float h0, float h1, float h2, float innov, float R) {
  const float ph0 = h0*p00_ + h1*p01_ + h2*p02_;
  const float ph1 = h0*p01_ + h1*p11_ + h2*p12_;
  const float ph2 = h0*p02_ + h1*p12_ + h2*p22_;
  const float S   = h0*ph0 + h1*ph1 + h2*ph2 + R;
  if (S < 1e-9f) return;

  const float k0 = ph0 / S;
  const float k1 = ph1 / S;
  const float k2 = ph2 / S;

  x_   += k0 * innov;
  y_   += k1 * innov;
  yaw_ += k2 * innov;

  p00_ -= k0 * ph0;
  p01_ -= k0 * ph1;
  p02_ -= k0 * ph2;
  p11_ -= k1 * ph1;
  p12_ -= k1 * ph2;
  p22_ -= k2 * ph2;

  p00_ = fmaxf(p00_, 1.0f);
  p11_ = fmaxf(p11_, 1.0f);
  p22_ = fmaxf(p22_, 1e-6f);
}

// ── AprilTag observation ───────────────────────────────────────────────────────
// Converts a tag hit into (z_x, z_y, z_yaw) observations and per-axis noise.
// Noise scales with distance^2 and oblique angle^2 so close frontal views are
// treated as near-absolute fixes and far/oblique views are weighted lightly.
bool PoseService::getTagObs_(const TagObs& obs, float& z_x, float& z_y, float& z_yaw,
                              float& r_pos, float& r_yaw) {
  const float az        = obs.az_deg      * ((float)M_PI / 180.0f);
  const float angle_rad = obs.tag_yaw_deg * ((float)M_PI / 180.0f);
  const float dist      = obs.dist_mm;

  const float tbx = dist * cosf(az);
  const float tby = -dist * sinf(az);

  const uint8_t id = obs.id;

  // Snapshot tag state under mutex (defineTag/resetMap may run on Core 0 concurrently).
  portENTER_CRITICAL(&mapMux_);
  const bool     known   = tagMap_[id].known;
  const uint8_t  nKnown  = nKnownTags_;
  const TagWorld cached  = tagMap_[id];
  portEXIT_CRITICAL(&mapMux_);

  if (!known) {
    // Unknown tag: learn its world position using the current EKF yaw estimate.
    if (nKnown > 0 && (p00_ > kTagLearnMaxP || p11_ > kTagLearnMaxP)) return false;

    const float c = cosf(yaw_), s = sinf(yaw_);
    const TagWorld newTag = { x_ + c*tbx - s*tby,
                              y_ + s*tbx + c*tby,
                              yaw_ - angle_rad + (float)M_PI, true };
    portENTER_CRITICAL(&mapMux_);
    tagMap_[id] = newTag;
    nKnownTags_++;
    portEXIT_CRITICAL(&mapMux_);
    Serial.printf("[Pose] tag %u learned at world (%.0f, %.0f) face=%.1f deg\n",
                  id, newTag.x_mm, newTag.y_mm,
                  newTag.face_yaw_rad * (180.0f / (float)M_PI));
    return false;
  }

  // z_yaw: heading derived from tag face direction and observed tag rotation.
  // Used ONLY for the heading correction step in run(); not used for position.
  // The R matrix mixes heading and lateral viewing angle for oblique views, making
  // z_yaw unreliable for position when the camera is not near the perpendicular.
  z_yaw = cached.face_yaw_rad + angle_rad - (float)M_PI;
  while (z_yaw >  (float)M_PI) z_yaw -= 2.0f * (float)M_PI;
  while (z_yaw < -(float)M_PI) z_yaw += 2.0f * (float)M_PI;

  // Position: use current EKF yaw (IMU, anchored by frontal views) for all views.
  // Once heading is correct, az_deg and dist_mm give accurate position at any angle.
  const float c = cosf(yaw_), s = sinf(yaw_);
  z_x = cached.x_mm - (c*tbx - s*tby);
  z_y = cached.y_mm - (s*tbx + c*tby);

  const float angle2 = angle_rad * angle_rad;
  const float dist2  = dist * dist;
  r_pos = kRTagPosBase + kRTagPosDist * dist2 + kRTagPosAng * angle2;
  r_yaw = kRTagYawBase + kRTagYawDist * dist2 + kRTagYawAng * angle2;
  return true;
}

// ── Main loop ─────────────────────────────────────────────────────────────────
void PoseService::run() {
  TickType_t last   = xTaskGetTickCount();
  uint32_t   prevMs = millis();

  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(20)); // 50 Hz

    const uint32_t nowMs = millis();
    const float dt = (float)(nowMs - prevMs) * 0.001f;
    prevMs = nowMs;
    if (dt <= 0.0f || dt > 0.5f) continue;

    // ── 1. Predict — tag-only mode ────────────────────────────────────────────
    // Tags are the sole source of pose. Process noise sized so EKF converges
    // within ~1 second when robot moves but still smooths per-frame noise.
    // 2000 mm²/s → steady-state σ ≈ 11mm (vs 14mm tag noise → light smoothing)
    predict_(0.0f, 0.0f, 0.0f, 2000.0f * dt, kQYawCmd * dt);

    // ── 3. AprilTag updates ───────────────────────────────────────────────────
    TagObs obs[4];
    uint8_t nObs;
    portENTER_CRITICAL(&tagMux_);
    nObs = nLatestTags_;
    for (uint8_t i = 0; i < nObs; i++) obs[i] = latestTags_[i];
    nLatestTags_ = 0;
    portEXIT_CRITICAL(&tagMux_);

    if (nObs > 0 && locSweepActive_) sweepTagSeen_ = true;

    // Pass 1: position-based yaw update from all known tags.
    // z_yaw = atan2(tag_y - y_, tag_x - x_) - az_rad:
    //   the heading that would place the observed tag at its detected azimuth.
    // This works at any viewing angle — no frontal gate needed.
    // r_yaw = (Px + Py) / dist² + base_noise:
    //   when position is uncertain the yaw nudge is gentle; as position
    //   converges both quantities tighten together.
    bool yawUpdated = false;

    for (uint8_t i = 0; i < nObs; i++) {
      const uint8_t id = obs[i].id;
      portENTER_CRITICAL(&mapMux_);
      const bool     known  = tagMap_[id].known;
      const TagWorld cached = tagMap_[id];
      portEXIT_CRITICAL(&mapMux_);
      if (!known) continue;

      const float az_rad    = obs[i].az_deg     * ((float)M_PI / 180.0f);
      const float angle_rad = obs[i].tag_yaw_deg * ((float)M_PI / 180.0f);
      const float dist2     = obs[i].dist_mm * obs[i].dist_mm;

      // Face-direction yaw: derived from the tag's known face angle and observed
      // tag_yaw_deg.  Position-independent — works even when x_,y_ are far wrong.
      // This is the primary bootstrap mechanism for absolute heading.
      if (fabsf(obs[i].tag_yaw_deg) < 65.0f) {
        float fy = cached.face_yaw_rad + angle_rad - (float)M_PI;
        while (fy >  (float)M_PI) fy -= 2.0f * (float)M_PI;
        while (fy < -(float)M_PI) fy += 2.0f * (float)M_PI;
        float fy_innov = fy - yaw_;
        while (fy_innov >  (float)M_PI) fy_innov -= 2.0f * (float)M_PI;
        while (fy_innov < -(float)M_PI) fy_innov += 2.0f * (float)M_PI;
        if (fabsf(fy_innov) > 0.52f && p22_ < 0.01f) {
          p22_ = 0.5f;
          Serial.printf("[Pose] face-yaw reset: tag%u innov=%.1fdeg\n", id,
                        fy_innov * (180.f / (float)M_PI));
        }
        const float angle2  = angle_rad * angle_rad;
        const float r_face  = kRTagYawBase + kRTagYawAng * angle2; // no dist term — heading noise is angle-driven, not range-driven
        updateScalar_(0.0f, 0.0f, 1.0f, fy_innov, r_face);
        yawUpdated = true;
      }

    }

    if (yawUpdated && odom_) {
      odom_->correctYawRad(yaw_);
      prevImuYaw_ = yaw_;  // re-sync so next delta doesn't include the correction jump
    }

    // Pass 2: position update from each tag.
    //
    // Strategy: distance-only update first (yaw-independent, always safe), then
    // azimuth-based update when the az-derived position projects INSIDE the arena.
    //
    // The az-based update is essential when facing a single wall: the distance
    // circle has two solutions at similar x but very different y (or vice-versa),
    // and the scan-to-map cannot resolve this when the LiDAR FoV doesn't cover
    // the perpendicular walls.  The azimuth directly pins which solution is correct.
    //
    // Safety gate: if az+yaw projects outside arena bounds, skip az update only
    // (distance update still applies).  This handles the "yaw very wrong" case
    // without breaking normal operation.
    for (uint8_t i = 0; i < nObs; i++) {
      const uint8_t did = obs[i].id;
      portENTER_CRITICAL(&mapMux_);
      const bool     dknown  = tagMap_[did].known;
      const TagWorld dcached = tagMap_[did];
      portEXIT_CRITICAL(&mapMux_);
      if (!dknown) continue;

      // Tag-only mode: azimuth position fix only.
      // Compute (z_x, z_y) from tag face direction + observed az/dist, then
      // apply directly. No distance-only update — it fights the azimuth fix
      // when yaw is wrong.
      if (fabsf(obs[i].tag_yaw_deg) >= 65.0f) continue; // oblique — skip

      float z_x = 0, z_y = 0, z_yaw_unused = 0, r_pos = kRTagPosBase, r_yaw_unused = 0;
      if (!getTagObs_(obs[i], z_x, z_y, z_yaw_unused, r_pos, r_yaw_unused)) continue;

      const float azMargin = 80.0f;
      if (z_x < -azMargin || z_x > arenaW_ + azMargin ||
          z_y < -azMargin || z_y > arenaH_ + azMargin) continue; // outside arena

      updateScalar_(1.0f, 0.0f, 0.0f, z_x - x_, r_pos);
      updateScalar_(0.0f, 1.0f, 0.0f, z_y - y_, r_pos);
    }

    // ── 3b. Scan-to-map EKF updates ──────────────────────────────────────────
    // Ray-casts each near-horizontal LiDAR beam against the rectangular arena model.
    // Unlike the old wall-hit detector, this applies a full [x,y,yaw] Jacobian so
    // wall geometry corrects heading even without AprilTags visible.
    applyToFScanMatchUpdate_();

    // Soft arena-wall constraints: virtual position measurements at each edge.
    // When the EKF drifts outside the physical arena the innovations pull it
    // back in, breaking wrong-attractor solutions that only exist outside bounds.
    if (arenaW_ > 0.0f) {
      if (x_ < 0.0f)       updateScalar_(1.0f, 0.0f, 0.0f, 0.0f - x_,      kRArenaWall);
      if (x_ > arenaW_)    updateScalar_(1.0f, 0.0f, 0.0f, arenaW_ - x_,   kRArenaWall);
    }
    if (arenaH_ > 0.0f) {
      if (y_ < 0.0f)       updateScalar_(0.0f, 1.0f, 0.0f, 0.0f - y_,      kRArenaWall);
      if (y_ > arenaH_)    updateScalar_(0.0f, 1.0f, 0.0f, arenaH_ - y_,   kRArenaWall);
    }

    // ── Auto-recalibrate IMU bias once EKF converges ─────────────────────────
    // When position and yaw are well-known and stable for 5 s, the robot is
    // likely stationary — recalibrate the gyro bias so any contamination from
    // a reinit-during-motion is corrected.
    if (odom_ && p00_ < 500.0f && p11_ < 500.0f && p22_ < 0.003f) {
      if (ekfStableMs_ == 0) ekfStableMs_ = nowMs;
      else if (nowMs - ekfStableMs_ > 5000) {
        Serial.println("[Pose] EKF converged — recalibrating IMU bias");
        odom_->recalibrateBias();
        ekfStableMs_ = 0;
      }
    } else {
      ekfStableMs_ = 0;
    }

    // ── Localization sweep ────────────────────────────────────────────────────
    // Rotates the robot slowly so scan-to-map and tag updates can converge both
    // position and absolute heading from a completely unknown starting state.
    if (locSweepActive_) {
      // Accumulate total rotation magnitude (unsigned, so CW and CCW both count)
      float dYaw = yaw_ - sweepLastYaw_;
      while (dYaw >  (float)M_PI) dYaw -= 2.0f * (float)M_PI;
      while (dYaw < -(float)M_PI) dYaw += 2.0f * (float)M_PI;
      sweepAccumYaw_ += fabsf(dYaw);
      sweepLastYaw_ = yaw_;

      const bool fullSweep = (sweepAccumYaw_ >= kSweepMaxYaw);
      // "Converged" requires tight covariance AND a good scan-to-map fit AND
      // at least one AprilTag anchor during the sweep.  Tight covariance alone
      // just means the EKF is confident — it may be confident but WRONG (local
      // minimum).  The scan residual check and tag requirement prevent false
      // positives; without them the UI shows "converged ✓" for a wrong pose.
      const bool goodFit = (lastScanBeams_ >= kSweepResidMinB &&
                            lastScanMeanResid_ < kSweepResidOkMm);
      const bool ekfTight = (p22_ < kSweepDoneP22 &&
                             p00_ < kSweepDonePos && p11_ < kSweepDonePos &&
                             sweepAccumYaw_ > kSweepMinYaw);
      const bool converged = ekfTight && goodFit && sweepTagSeen_;

      if (converged || fullSweep) {
        locSweepActive_ = false;
        if (motion_) motion_->setCommandVec(0.0f, 0.0f, 0.0f);
        Serial.printf("[Pose] Loc sweep %s: x=%.0f y=%.0f yaw=%.1fdeg "
                      "Pyaw=%.5f resid=%.1fmm beams=%d tag=%d accum=%.0fdeg\n",
                      converged ? "CONVERGED" : "done-no-fix",
                      x_, y_, yaw_ * (180.0f / (float)M_PI), p22_,
                      lastScanMeanResid_, lastScanBeams_, sweepTagSeen_ ? 1 : 0,
                      sweepAccumYaw_ * (180.0f / (float)M_PI));
        if (!converged) {
          // Reset to wide variance so a subsequent sweep or tag sighting can
          // re-initialize rather than staying locked to the wrong attractor.
          p00_ = kTagLearnMaxP; p01_ = 0.0f; p02_ = 0.0f;
          p11_ = kTagLearnMaxP; p12_ = 0.0f;
        }
        if (sweepDoneCb_) sweepDoneCb_(converged);
      } else {
        // Keep rotating slowly — heading hold is bypassed because this IS intentional rotation
        if (motion_) motion_->setCommandVec(0.0f, 0.0f, kLocSweepRotSpeed);
      }
    }


    // ── 4. Publish ────────────────────────────────────────────────────────────
    Pose2D updated = bus_->get();
    updated.x_mm   = x_;
    updated.y_mm   = y_;
    updated.yaw_rad = yaw_;   // publish EKF yaw so tag-corrected heading is broadcast
    bus_->update(updated);
  }
}

// ── rayCastRect ───────────────────────────────────────────────────────────────
// Casts a ray from (x,y) in direction dir_rad against a rectangular arena
// (0..arenaW_, 0..arenaH_).  Returns the distance to the nearest wall, or NAN
// if no valid hit within 6000 mm.  *hitXWall is set true if the hit wall is a
// constant-x wall (left or right), false for a constant-y wall (top or bottom).
float PoseService::rayCastRect_(float x, float y, float dir_rad, bool* hitXWall) const {
  const float cd = cosf(dir_rad), sd = sinf(dir_rad);
  float t = 6001.0f;
  bool  xw = false;

  if (cd >  1e-4f) { const float tx = (arenaW_ - x) / cd; if (tx > 0.f && tx < t) { t = tx; xw = true;  } }
  if (cd < -1e-4f) { const float tx = (0.f     - x) / cd; if (tx > 0.f && tx < t) { t = tx; xw = true;  } }
  if (sd >  1e-4f) { const float ty = (arenaH_ - y) / sd; if (ty > 0.f && ty < t) { t = ty; xw = false; } }
  if (sd < -1e-4f) { const float ty = (0.f     - y) / sd; if (ty > 0.f && ty < t) { t = ty; xw = false; } }

  if (hitXWall) *hitXWall = xw;
  return (t < 6000.f) ? t : NAN;
}

// ── Scan-to-map EKF updates ───────────────────────────────────────────────────
// For each near-horizontal LiDAR beam, ray-casts against the rectangular arena
// model and applies a full [x,y,yaw] Jacobian EKF scalar update.  This provides:
//   • Position corrections from up to 24 beams per scan (vs. at most 8 in the old code)
//   • Yaw corrections from wall geometry (impossible with the old scalar x/y approach)
//   • Convergence from any starting pose without needing an AprilTag
//
// Jacobian derivation for a beam hitting wall W:
//   R_exp = t = distance to wall along beam direction d = (yaw + az_col + yaw_off)
//   x-wall: R = (W_x - x)/cd   →  dR/dx = -1/cd,  dR/dy = 0,  dR/dyaw = -R*sd/cd
//   y-wall: R = (W_y - y)/sd   →  dR/dx = 0,       dR/dy = -1/sd, dR/dyaw = -R*cd/sd
//
// Yaw Jacobian is applied only when |cos(incidence)| >= kScanYawMinCos (≈60° from
// wall normal) to avoid numerically large h2 terms from tangent beams.
void PoseService::applyToFScanMatchUpdate_() {
  if (!lidar_ || arenaW_ <= 0.0f || arenaH_ <= 0.0f) return;
  // Yaw must be established by a tag fix before scan-to-map runs.
  // With wrong yaw the ray-casts go in wrong directions and corrupt position.
  if (p22_ > 0.1f) return;

  const uint32_t gen = lidar_->generation();
  if (gen == lastScanMatchGen_) return;   // same scan already processed
  lastScanMatchGen_ = gen;

  uint16_t grid[64];
  if (!lidar_->getGrid(grid)) return;

  const float yaw_off = lidar_->lib().calibration().yaw_off_deg * ((float)M_PI / 180.0f);

  // Use a wider innovation gate while position uncertainty is large (startup / kidnap)
  // so the EKF can converge to the correct position from any starting point.
  const bool wideGate = (p00_ > 50000.0f || p11_ > 50000.0f);
  const float gate    = wideGate ? kScanMatchGateWideMm : kScanMatchGateMm;

  float sumResid    = 0.0f;
  int   nGoodBeams  = 0;
  int   nAttempted  = 0;   // ALL valid-range beams tried, including those rejected by gate
  float sumResidAll = 0.0f;
  lastScanBeams_     = 0;
  lastScanMeanResid_ = 999.f;

  // Rows 2-4: elevations roughly ±3-11° — close enough to horizontal to hit walls.
  for (int row = 2; row <= 4; row++) {
    const float el = lidar_->lib().elevationForRowRad(row);
    if (fabsf(el) > 0.22f) continue;
    const float cel = cosf(el);

    for (int col = 0; col < 8; col++) {
      const uint16_t rm16 = grid[row * 8 + col];
      if (rm16 < 80 || rm16 > 2200) continue;
      const float rm    = (float)rm16;
      const float rm_h  = rm * cel;  // horizontal projection of measured range

      // Beam direction in world frame
      const float beam_dir = yaw_ + yaw_off + lidar_->lib().azimuthForColumnRad(col);
      const float cd = cosf(beam_dir), sd = sinf(beam_dir);

      // Ray-cast expected range from current EKF pose
      bool hitXWall;
      const float R_exp = rayCastRect_(x_, y_, beam_dir, &hitXWall);
      if (!isfinite(R_exp) || R_exp < 10.0f) continue;

      const float innov = rm_h - R_exp;
      nAttempted++;
      sumResidAll += fabsf(innov);
      if (fabsf(innov) > gate) continue;

      // Build Jacobian [h0, h1, h2] = [dR/dx, dR/dy, dR/dyaw]
      float h0, h1, h2;
      if (hitXWall) {
        if (fabsf(cd) < 0.1f) continue;  // nearly tangent to x-wall — skip
        h0 = -1.0f / cd;
        h1 = 0.0f;
        // Yaw Jacobian only for good incidence angle to avoid large h2 magnitudes
        h2 = (fabsf(cd) >= kScanYawMinCos) ? (-R_exp * sd / cd) : 0.0f;
      } else {
        if (fabsf(sd) < 0.1f) continue;  // nearly tangent to y-wall — skip
        h0 = 0.0f;
        h1 = -1.0f / sd;
        h2 = (fabsf(sd) >= kScanYawMinCos) ? (-R_exp * cd / sd) : 0.0f;
      }

      const float R_noise = kRScanBase + kRScanDist * rm * rm;
      updateScalar_(h0, h1, h2, innov, R_noise);

      sumResid += fabsf(innov);
      nGoodBeams++;
    }
  }

  // Persist quality metrics — use ALL attempted beams for mean residual.
  // Using only gate-passing beams is misleading at wrong attractors: most beams
  // are rejected (large residuals) but the few that coincidentally pass give a
  // falsely low mean.  The all-beam mean exposes this.
  lastScanBeams_     = nGoodBeams;
  lastScanMeanResid_ = (nAttempted > 0) ? sumResidAll / nAttempted : 999.f;

  // When scan match is strong (low residuals, many beams) and EKF yaw is already
  // reasonably converged, anchor the IMU heading offset to the wall-derived yaw.
  // This corrects IMU drift continuously without needing an AprilTag visible.
  if (odom_ && nGoodBeams >= kScanHeadingMinBeams &&
      sumResid / nGoodBeams < kScanHeadingLockMm && p22_ < 0.01f) {
    odom_->correctYawRad(yaw_);
    prevImuYaw_ = yaw_;
  }
}

// ── ToF scan-to-scan range-flow odometry ──────────────────────────────────────
// Returns world-frame displacement for use as the EKF PREDICTION step.
//
// Solves a 2x2 least-squares:  dr_h[col] = cos(az)*dx_b + sin(az)*dy_b
// using only row 3 (smallest positive elevation, closest to horizontal).
//
// Rotation compensation: when the robot rotates between scans, each current-frame
// beam column is looking at a different wall feature than the same column in the
// previous frame.  We compensate by interpolating the reference scan at the
// rotated azimuth (azimuth + dyaw) so we always compare the same wall feature.
// This removes the old hard 7° gate — scan-flow now works continuously through
// moderate spins up to ~25° between scans.
bool PoseService::computeToFOdometry_(float& dx_w_out, float& dy_w_out) {
  if (!lidar_) return false;

  const uint32_t newGen = lidar_->generation();
  uint16_t curr[64];
  if (!lidar_->getGrid(curr) || newGen == prevScan_.gen) return false;

  const float yaw = yaw_;

  auto resetRef = [&]() {
    memcpy(prevScan_.ranges, curr, sizeof(curr));
    prevScan_.yaw_rad = yaw;
    prevScan_.gen     = newGen;
    prevScan_.valid   = true;
  };

  if (!prevScan_.valid) { resetRef(); return false; }

  float dyaw = yaw - prevScan_.yaw_rad;
  while (dyaw >  (float)M_PI) dyaw -= 2.0f * (float)M_PI;
  while (dyaw < -(float)M_PI) dyaw += 2.0f * (float)M_PI;
  // Soft gate at ~25°: beyond this the linear interpolation degrades too much.
  if (fabsf(dyaw) > 0.44f) { resetRef(); return false; }

  const int   row = 3;
  const float el  = lidar_->lib().elevationForRowRad(row);
  if (el < 0.0f) { resetRef(); return false; }  // skip downward-pointing rows

  // Pre-compute per-column azimuths and derive uniform step for interpolation.
  float azimuths[8];
  for (int c = 0; c < 8; c++) azimuths[c] = lidar_->lib().azimuthForColumnRad(c);
  const float az_step = (azimuths[7] - azimuths[0]) / 7.0f;  // may be negative
  if (fabsf(az_step) < 1e-5f) { resetRef(); return false; }

  const float cel = cosf(el);
  float sum_cc=0, sum_ss=0, sum_cs=0, sum_cdr=0, sum_sdr=0;
  int   count = 0;

  for (int col = 0; col < 8; col++) {
    const int   idx_curr = row * 8 + col;
    const float r1 = (float)curr[idx_curr];
    if (r1 < 50.f || r1 > 2500.f) continue;

    // Find the reference-scan range corresponding to the same world direction.
    // The current beam at az_col looks in world direction (yaw + az_col).
    // In the previous scan (at yaw - dyaw), the same world direction was at
    // body azimuth (az_col + dyaw).
    const float az_in_ref = azimuths[col] + dyaw;
    const float fidx      = (az_in_ref - azimuths[0]) / az_step;
    const int   c0        = (int)floorf(fidx);
    const float frac      = fidx - c0;
    if (c0 < 0 || c0 >= 7) continue;  // rotated out of scan range — skip

    const float r0a = (float)prevScan_.ranges[row * 8 + c0];
    const float r0b = (float)prevScan_.ranges[row * 8 + c0 + 1];
    if (r0a < 50.f || r0a > 2500.f || r0b < 50.f || r0b > 2500.f) continue;
    const float r0 = (1.0f - frac) * r0a + frac * r0b;

    const float dr = r0 - r1;
    if (fabsf(dr) > 80.f) continue;   // large change → obstacle, not motion

    const float dr_h = dr * cel;
    const float az   = azimuths[col];
    const float ca = cosf(az), sa = sinf(az);
    sum_cc  += ca*ca;  sum_ss  += sa*sa;  sum_cs  += ca*sa;
    sum_cdr += ca*dr_h;  sum_sdr += sa*dr_h;
    count++;
  }

  if (count < kMinToFBeams) { resetRef(); return false; }

  const float det = sum_cc*sum_ss - sum_cs*sum_cs;
  if (fabsf(det) < 0.01f) { resetRef(); return false; }

  const float dx_b = (sum_ss*sum_cdr - sum_cs*sum_sdr) / det;
  const float dy_b = (sum_cc*sum_sdr - sum_cs*sum_cdr) / det;
  if (fabsf(dx_b) > 150.f || fabsf(dy_b) > 150.f) { resetRef(); return false; }

  const float yaw_avg = prevScan_.yaw_rad + dyaw * 0.5f;
  const float cw = cosf(yaw_avg), sw = sinf(yaw_avg);
  dx_w_out = cw*dx_b - sw*dy_b;
  dy_w_out = sw*dx_b + cw*dy_b;

  resetRef();
  return true;
}
