// BlobService.h
// Binary blob detector on QQVGA (160×120) grayscale frames.
// Uses Otsu's threshold + 4-connected union-find to find up to kMaxBlobs blobs.
// Sorted by area (largest first).  Zero CPU when not the active consumer.
#pragma once
#include <Arduino.h>
#include <functional>
#include "../core/ICVConsumer.h"

class BlobService : public ICVConsumer {
public:
    static constexpr int kW = 160;
    static constexpr int kH = 120;
    static constexpr int kMaxBlobs = 16;

    struct Blob {
        int   cx, cy;         // centroid (pixel coords)
        int   area;           // pixel count
        int   x0, y0, x1, y1; // bounding box
        float aspect;         // (x1-x0) / (y1-y0)
    };

    // Callback invoked from the camera task after each frame.
    using ResultCb = std::function<void(const Blob* blobs, int count)>;
    void setCallback(ResultCb cb) { resultCb_ = std::move(cb); }

    // Threshold override (0 = auto Otsu per frame).
    void setThreshold(uint8_t thr) { fixedThr_ = thr; autoThr_ = (thr == 0); }

    bool begin();

    void onFrame(camera_fb_t* fb) override;
    void onActivate()             override;
    void onDeactivate()           override;

private:
    uint8_t otsu_(const uint8_t* img, int n);

    ResultCb resultCb_{};
    uint8_t  fixedThr_ = 0;
    bool     autoThr_  = true;

    // Work buffers allocated in PSRAM on activate, freed on deactivate.
    uint8_t* binary_ = nullptr;  // 160*120 binary image
    int*     label_  = nullptr;  // 160*120 label image
    int*     parent_ = nullptr;  // union-find parent array (size = kW*kH)
    int*     rank_   = nullptr;  // union-find rank array
};
