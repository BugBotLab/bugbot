// ContourService.cpp — Sobel edge detection on QQVGA grayscale frames.
#include "ContourService.h"
#include <esp_heap_caps.h>
#include <math.h>
#include <string.h>

bool ContourService::begin() { return true; }

void ContourService::onActivate() {
    const int N = kW * kH;
    gradX_ = (int16_t*)heap_caps_malloc(N * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    gradY_ = (int16_t*)heap_caps_malloc(N * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    edges_ = (uint8_t*)heap_caps_malloc(N,                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!gradX_ || !gradY_ || !edges_) {
        Serial.println("[Contour] PSRAM alloc failed");
    } else {
        Serial.println("[Contour] ready");
    }
}

void ContourService::onDeactivate() {
    heap_caps_free(gradX_); gradX_ = nullptr;
    heap_caps_free(gradY_); gradY_ = nullptr;
    heap_caps_free(edges_); edges_ = nullptr;
}

void ContourService::onFrame(camera_fb_t* fb) {
    if (!gradX_ || !gradY_ || !edges_) return;

    const uint8_t* src = fb->buf;

    // 3×3 Sobel — skip 1-pixel border to avoid bounds checks
    // Sobel Gx: [-1 0 +1; -2 0 +2; -1 0 +1]
    // Sobel Gy: [-1 -2 -1; 0 0 0; +1 +2 +1]
    int edgeCount = 0;
    uint32_t angleAccumX = 0, angleAccumY = 0; // for dominant direction

    for (int y = 1; y < kH - 1; y++) {
        const uint8_t* row0 = src + (y - 1) * kW;
        const uint8_t* row1 = src + y * kW;
        const uint8_t* row2 = src + (y + 1) * kW;
        uint8_t* out = edges_ + y * kW;

        // Border pixels left/right are zero
        out[0] = 0;
        out[kW - 1] = 0;

        for (int x = 1; x < kW - 1; x++) {
            const int gx = -row0[x-1] + row0[x+1]
                           - 2*row1[x-1] + 2*row1[x+1]
                           - row2[x-1] + row2[x+1];
            const int gy = -row0[x-1] - 2*row0[x] - row0[x+1]
                           + row2[x-1] + 2*row2[x] + row2[x+1];

            // Approximate magnitude: |gx| + |gy| (cheap, avoids sqrt)
            const int mag = (gx < 0 ? -gx : gx) + (gy < 0 ? -gy : gy);
            if (mag > edgeThr_) {
                out[x] = 255;
                edgeCount++;
                // Accumulate gradient direction for dominant angle
                angleAccumX += (uint32_t)(gx < 0 ? -gx : gx);
                angleAccumY += (uint32_t)(gy < 0 ? -gy : gy);
            } else {
                out[x] = 0;
            }
        }
    }
    // Zero top/bottom border rows
    memset(edges_, 0, kW);
    memset(edges_ + (kH - 1) * kW, 0, kW);

    float dominantAngle = 0.0f;
    if (angleAccumX || angleAccumY) {
        dominantAngle = atan2f((float)angleAccumY, (float)angleAccumX) * (180.0f / M_PI);
    }

    if (resultCb_) {
        const EdgeResult r = { edges_, edgeCount, dominantAngle };
        resultCb_(r);
    }
}
