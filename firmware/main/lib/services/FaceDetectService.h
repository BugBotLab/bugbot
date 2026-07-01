// FaceDetectService.h
// ICVConsumer that runs the official esp-dl MSRMNP two-stage face detector.
//
// Architecture:
//   CameraService (RGB565 QQVGA) -> onFrame() -> MSRMNP::run() -> ResultCb
//   Stage 1 MSR: coarse multi-scale candidates
//   Stage 2 MNP: per-candidate refinement + 5-point landmarks
//
// Camera mode: PIXFORMAT_RGB565, FRAMESIZE_QQVGA (160x120).
// CameraService reinits OV2640 automatically when switching from the default
// grayscale mode used by AprilTag/Blob/Contour.
//
// Model files (auto-included in SPIFFS image, placed in data/models/):
//   human_face_detect_msr_s8_v1.espdl
//   human_face_detect_mnp_s8_v1.espdl
#pragma once
#include <functional>
#include "../core/ICVConsumer.h"

class FaceDetectService : public ICVConsumer {
public:
    struct Face {
        int16_t x1, y1, x2, y2;    // bounding box in pixel coords
        float   score;              // detection confidence 0..1
        int16_t kp[10];            // landmarks [lx,ly, rx,ry, nx,ny, mlx,mly, mrx,mry]
    };
    static constexpr int kMaxFaces = 8;

    // Invoked after each frame that contains at least one face.
    // Called on the camera task — keep the callback short.
    using ResultCb = std::function<void(const Face* faces, int count)>;
    void setCallback(ResultCb cb) { resultCb_ = std::move(cb); }

    bool begin() { return true; }

    // Number of frames where inference actually ran (detector was loaded and a
    // frame arrived). Resets to 0 on onDeactivate(). Useful for diagnostics.
    int frameCount() const { return frameCount_; }

    // True if the MSRMNP detector was successfully allocated (model files found).
    bool detectorReady() const { return detector_ != nullptr; }

    // Request RGB565 colour frames — MSR_S8_V1 needs colour input.
    // OV2640 outputs RGB565 big-endian (high byte = RRRRRGGG first in memory).
    CameraMode cameraMode() const override {
        return { PIXFORMAT_RGB565, FRAMESIZE_QQVGA };
    }

    // onActivate: allocates the HumanFaceDetect detector (lazy-loads model on first run).
    // onDeactivate: deletes the detector and frees its NPU memory.
    void onActivate()   override;
    void onDeactivate() override;
    void onFrame(camera_fb_t* fb) override;

private:
    ResultCb resultCb_{};
    // HumanFaceDetect* stored as void* to avoid pulling heavy esp-dl headers
    // into every translation unit that includes this header.
    void* detector_ = nullptr;
    int   frameCount_ = 0;
};
