// AprilTagService.h
// ArucoLite (APRILTAG_16h5) marker detection implemented as an ICVConsumer.
// CameraService calls onFrame() for every captured frame; this class processes
// the grayscale pixels synchronously and broadcasts results over WebSocket.
//
// The Aruco detector (~25 kB) is allocated in PSRAM only while this service is
// the active consumer (onActivate → heap_caps_malloc, onDeactivate → free).
#pragma once
#include <Arduino.h>
#include <functional>
#include "../net/WebWS.h"
#include "../core/ICVConsumer.h"

class AprilTagService : public ICVConsumer {
public:
    // Store parameters and compute camera intrinsics.
    // tag_size_mm: physical side length of the tag (black border only).
    // fov_h_deg:   horizontal field-of-view in degrees (OV2640 ≈ 66°).
    bool begin(WebWS& ws,
               float tag_size_mm = 50.0f,
               float fov_h_deg   = 66.0f);

    // Optional in-process callback: invoked from the camera task on every frame
    // (even frames with zero detections, so callers can clear stale caches).
    using TagCallback = std::function<void(uint8_t, const WebWS::AprilTagHit*)>;
    void setTagCallback(TagCallback cb) { tagCallback_ = std::move(cb); }

    // ICVConsumer interface
    void onFrame(camera_fb_t* fb) override;
    void onActivate()             override;  // allocates Aruco detector
    void onDeactivate()           override;  // frees Aruco detector

private:
    WebWS*        ws_        = nullptr;
    float         tagSizeMm_ = 50.0f;
    float         fovH_deg_  = 66.0f;
    TagCallback   tagCallback_{};

    // Camera intrinsics for QQVGA (160×120), computed in begin().
    float camCx_ = 80.0f;
    float camCy_ = 60.0f;
    float fPx_   = 0.0f;

    // Aruco detector — allocated as raw PSRAM in onActivate(), freed in onDeactivate().
    // Typed as void* to keep ArucoLite.h out of the public header.
    void* arucoMem_ = nullptr;

    // Per-tag EMA smoothing state (16 tag IDs max).
    struct TagSmooth {
        float    az, dist, yaw;
        uint32_t lastSeenMs;
        bool     valid;
    };
    TagSmooth smooth_[16] = {};

    // Frame-rate / sequencing state.
    uint32_t seq_         = 0;
    uint32_t lastFpsMs_   = 0;
    uint32_t totalFrames_ = 0;
    uint32_t foundFrames_ = 0;
    uint32_t lastSnapshotMs_ = 0;
};
