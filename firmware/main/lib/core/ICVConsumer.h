// ICVConsumer.h
// Interface for all computer-vision consumers of camera frames.
//
// Each consumer declares the camera mode it needs via cameraMode().
// CameraService reads this on setConsumer() and reinitialises the OV2640
// automatically if the format or resolution changed.
//
// Lifecycle calls (onActivate / onDeactivate) happen from whatever task
// calls setConsumer() -- NOT the camera task -- so PSRAM alloc/free is safe.
#pragma once
#include <esp_camera.h>

// Camera configuration a consumer requests.
// CameraService reinits the OV2640 whenever the active consumer's mode
// differs from the currently configured mode.
struct CameraMode {
    pixformat_t pixformat = PIXFORMAT_GRAYSCALE;  // default: gray (AprilTag, Blob, Contour)
    framesize_t framesize = FRAMESIZE_QQVGA;       // default: 160x120

    bool operator==(const CameraMode& o) const {
        return pixformat == o.pixformat && framesize == o.framesize;
    }
    bool operator!=(const CameraMode& o) const { return !(*this == o); }
};

class ICVConsumer {
public:
    virtual ~ICVConsumer() = default;

    // Called for every captured frame. Must return quickly.
    // TinyMLService copies the frame and returns immediately; all others
    // process synchronously. The frame buffer is returned after this call.
    virtual void onFrame(camera_fb_t* fb) = 0;

    // Called when this consumer becomes active. Safe to allocate PSRAM here.
    virtual void onActivate()   {}

    // Called just before this consumer is deactivated. Safe to free PSRAM.
    virtual void onDeactivate() {}

    // Declare what camera format/resolution this consumer needs.
    // CameraService reinits the OV2640 if the mode differs from the current one.
    // Default matches the original BugBot config (grayscale QQVGA).
    virtual CameraMode cameraMode() const { return {}; }
};
