// FaceDetectService.cpp
//
// Uses esp-dl v3.x human_face_detect::MSRMNP — the official two-stage pipeline:
//   Stage 1 (MSR):  coarse multi-scale candidate detection
//   Stage 2 (MNP):  per-candidate refinement + landmark regression
//
// Model files required on LittleFS at /littlefs/models/:
//   human_face_detect_msr_s8_v1.espdl   (MSR stage, ~60 KB)
//   human_face_detect_mnp_s8_v1.espdl   (MNP stage, ~127 KB)
//
// LittleFS mounts the "spiffs" flash partition at /littlefs (VFS).
// The LittleFS image is built from data/ by littlefs_create_partition_image
// in CMakeLists.txt and flashed alongside the app.
#include "FaceDetectService.h"
#include <Arduino.h>
#include <stdio.h>
#include <esp_camera.h>

// esp-dl image types (img_t, pix_type_t)
#include "dl_image_define.hpp"

// Official two-stage face detector — human_face_detect.hpp/.cpp in this directory.
// CONFIG_ macros controlling model location are set in CMakeLists.txt via
// target_compile_definitions so both this TU and human_face_detect.cpp agree.
#include "human_face_detect.hpp"

// ── lifecycle ─────────────────────────────────────────────────────────────────

void FaceDetectService::onActivate() {
    // Re-enable AEC/AGC — CameraService locks them off immediately after init
    // to prevent hunting in AprilTag/blob pipelines, but face detection needs
    // auto-exposure to converge for a proper image.
    sensor_t* s = esp_camera_sensor_get();
    if (s) {
        s->set_exposure_ctrl(s, 1);
        s->set_gain_ctrl(s, 1);
        Serial.println("[Face] AEC/AGC enabled for face detection");
    }

    if (detector_) return;

    const char* msr = "/littlefs/models/human_face_detect_msr_s8_v1.espdl";
    const char* mnp = "/littlefs/models/human_face_detect_mnp_s8_v1.espdl";

    FILE* f = fopen(msr, "rb");
    if (!f) {
        Serial.printf("[Face] model not found: %s\n", msr);
        Serial.println("[Face] flash LittleFS image with: build.ps1 -Flash");
        return;
    }
    fclose(f);

    FILE* f2 = fopen(mnp, "rb");
    if (!f2) {
        Serial.printf("[Face] MNP model not found: %s\n", mnp);
        return;
    }
    fclose(f2);

    Serial.println("[Face] loading MSRMNP face detection models...");
    detector_ = new human_face_detect::MSRMNP(
        "human_face_detect_msr_s8_v1.espdl", 0.3f, 0.5f,
        "human_face_detect_mnp_s8_v1.espdl", 0.3f, 0.5f
    );
    Serial.println("[Face] detector ready");
}

void FaceDetectService::onDeactivate() {
    if (detector_) {
        delete static_cast<human_face_detect::MSRMNP*>(detector_);
        detector_ = nullptr;
    }
    frameCount_ = 0;
    Serial.println("[Face] detector deactivated");
}

// ── frame processing ──────────────────────────────────────────────────────────

void FaceDetectService::onFrame(camera_fb_t* fb) {
    if (!detector_ || !fb || !fb->buf) return;
    frameCount_++;

    dl::image::img_t img;
    img.data     = fb->buf;
    img.width    = (uint16_t)fb->width;
    img.height   = (uint16_t)fb->height;
    img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565BE;

    auto& results = static_cast<human_face_detect::MSRMNP*>(detector_)->run(img);

    if (!resultCb_) return;

    if (results.empty()) {
        resultCb_(nullptr, 0);
        return;
    }

    Face faces[kMaxFaces];
    int  count = 0;
    for (const auto& r : results) {
        if (count >= kMaxFaces) break;
        Face& f = faces[count++];
        f.x1    = (r.box.size() >= 4) ? (int16_t)r.box[0] : 0;
        f.y1    = (r.box.size() >= 4) ? (int16_t)r.box[1] : 0;
        f.x2    = (r.box.size() >= 4) ? (int16_t)r.box[2] : 0;
        f.y2    = (r.box.size() >= 4) ? (int16_t)r.box[3] : 0;
        f.score = r.score;
        for (int k = 0; k < 10; k++)
            f.kp[k] = ((int)r.keypoint.size() > k) ? (int16_t)r.keypoint[k] : 0;
    }
    resultCb_(faces, count);
}
