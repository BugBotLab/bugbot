// ContourService.h
// Sobel edge detection on QQVGA (160×120) grayscale frames.
// Produces a thresholded edge map (1 B/px) and the Hough-line dominant angle.
// Zero CPU when not the active consumer.
#pragma once
#include <Arduino.h>
#include <functional>
#include "../core/ICVConsumer.h"

class ContourService : public ICVConsumer {
public:
    static constexpr int kW = 160;
    static constexpr int kH = 120;

    struct EdgeResult {
        const uint8_t* edgeMap;  // kW*kH pixels, 255=edge 0=background (PSRAM)
        int            edgeCount; // number of edge pixels above threshold
        float          dominantAngleDeg; // -90..90, dominant gradient direction
    };

    using ResultCb = std::function<void(const EdgeResult&)>;
    void setCallback(ResultCb cb) { resultCb_ = std::move(cb); }

    // Edge magnitude threshold (0–255).  Lower = more edges.
    void setThreshold(uint8_t thr) { edgeThr_ = thr; }

    bool begin();

    void onFrame(camera_fb_t* fb) override;
    void onActivate()             override;
    void onDeactivate()           override;

private:
    ResultCb resultCb_{};
    uint8_t  edgeThr_ = 40;

    // PSRAM work buffers
    int16_t* gradX_ = nullptr;  // kW*kH horizontal gradient
    int16_t* gradY_ = nullptr;  // kW*kH vertical gradient
    uint8_t* edges_ = nullptr;  // kW*kH thresholded output
};
