// CameraService.cpp
// Extracted from AprilTagService: all OV2640 hardware management lives here.
// Detection / processing lives in the individual CV service classes (ICVConsumer).
#include "CameraService.h"
#include <esp_camera.h>
#include <esp_cache.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <string.h>

// ── helpers ───────────────────────────────────────────────────────────────────

void CameraService::frameSizeDims(framesize_t fs, int& w, int& h) {
    switch (fs) {
        case FRAMESIZE_QQVGA: w = 160; h = 120; break;
        case FRAMESIZE_QVGA:  w = 320; h = 240; break;
        case FRAMESIZE_VGA:   w = 640; h = 480; break;
        case FRAMESIZE_SVGA:  w = 800; h = 600; break;
        default:              w = 160; h = 120; break;
    }
}

// ── camera init ───────────────────────────────────────────────────────────────

bool CameraService::initCamera_(const CameraMode& mode) {
    // Pre-warm XCLK: esp_camera_deinit() always calls sensor.reset() (COM7_SRST)
    // then ledc_stop().  The OV2640 needs several clock cycles after COM7_SRST to
    // complete its internal register reset before SCCB commands work.  Restarting
    // XCLK here and waiting 50 ms lets the sensor finish that reset sequence
    // before esp_camera_init() starts probing via SCCB.
    {
        ledc_timer_config_t lt = {};
        lt.duty_resolution = LEDC_TIMER_1_BIT;
        lt.freq_hz         = 24000000;
        lt.speed_mode      = LEDC_LOW_SPEED_MODE;
        lt.timer_num       = LEDC_TIMER_1;
        lt.clk_cfg         = LEDC_AUTO_CLK;
        ledc_timer_config(&lt);

        ledc_channel_config_t lc = {};
        lc.gpio_num   = 10;
        lc.speed_mode = LEDC_LOW_SPEED_MODE;
        lc.channel    = LEDC_CHANNEL_2;
        lc.timer_sel  = LEDC_TIMER_1;
        lc.duty       = 1;
        lc.hpoint     = 0;
        ledc_channel_config(&lc);
    }
    vTaskDelay(pdMS_TO_TICKS(50));   // OV2640 needs ~2 ms; 50 ms gives margin

    camera_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.ledc_channel    = LEDC_CHANNEL_2;
    cfg.ledc_timer      = LEDC_TIMER_1;
    cfg.pin_pwdn        = -1;
    cfg.pin_reset       = -1;
    cfg.pin_xclk        = 10;
    cfg.pin_sccb_sda    = 40;
    cfg.pin_sccb_scl    = 39;
    cfg.pin_d7          = 48;
    cfg.pin_d6          = 11;
    cfg.pin_d5          = 12;
    cfg.pin_d4          = 14;
    cfg.pin_d3          = 16;
    cfg.pin_d2          = 18;
    cfg.pin_d1          = 17;
    cfg.pin_d0          = 15;
    cfg.pin_vsync       = 38;
    cfg.pin_href        = 47;
    cfg.pin_pclk        = 13;
    cfg.xclk_freq_hz    = 24000000;         // OV2640 max XCLK
    cfg.pixel_format    = mode.pixformat;   // from consumer's cameraMode()
    cfg.frame_size      = mode.framesize;
    cfg.jpeg_quality    = 12;
    cfg.fb_count        = 2;                // double-buffer for continuous capture
    cfg.fb_location     = CAMERA_FB_IN_PSRAM;
    cfg.grab_mode       = CAMERA_GRAB_LATEST;

    if (esp_camera_init(&cfg) != ESP_OK) {
        Serial.println("[Cam] init failed");
        return false;
    }
    sensor_t* s = esp_camera_sensor_get();
    if (s) {
        s->set_hmirror(s, 1);       // XIAO camera is physically mirrored
        s->set_contrast(s, 2);
        s->set_sharpness(s, 2);
        s->set_exposure_ctrl(s, 0); // lock AEC — hunting degrades CV pipelines
        s->set_gain_ctrl(s, 0);
    }

    int w = 160, h = 120;
    frameSizeDims(mode.framesize, w, h);
    expW_ = w;
    expH_ = h;

    const char* fmtStr = (mode.pixformat == PIXFORMAT_GRAYSCALE) ? "GRAY"
                       : (mode.pixformat == PIXFORMAT_RGB565)    ? "RGB565"
                       :                                            "OTHER";
    Serial.printf("[Cam] %s %dx%d ready\n", fmtStr, w, h);
    return true;
}

// ── frame-capture task ────────────────────────────────────────────────────────

void CameraService::run() {
    vTaskDelay(pdMS_TO_TICKS(600));

    while (sleepReq_) {
        idle_ = true;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    idle_ = false;

    if (!cameraInited_) {
        if (!initCamera_(currentMode_)) {
            idle_ = true;
            vTaskSuspend(nullptr);
            return;
        }
        cameraInited_ = true;
    }

    for (;;) {
        if (sleepReq_ || g_cameraStreaming) {
            idle_ = true;
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // If a mode-switch reinit failed in setConsumer(), retry here.
        if (!cameraInited_) {
            idle_ = true;
            Serial.println("[Cam] retrying camera init...");
            if (initCamera_(currentMode_)) {
                cameraInited_ = true;
                Serial.println("[Cam] retry init OK");
            } else {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            continue;
        }

        idle_ = false;

        camera_fb_t* fb = esp_camera_fb_get(); // can block up to 4 s (FB_GET_TIMEOUT)

        // Re-check immediately after fb_get — it can block up to 4 s.
        // If suspend() was called while we were blocked, go idle now so
        // suspend() can safely call esp_camera_deinit() without deleting a live queue.
        if (sleepReq_) {
            if (fb) esp_camera_fb_return(fb);
            idle_ = true;
            continue;
        }

        if (!fb) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }

        // Reject frames that don't match the configured resolution — happens
        // for one or two frames immediately after a mode switch while the
        // OV2640 drains its internal FIFO.
        if ((int)fb->width != expW_ || (int)fb->height != expH_) {
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        // Camera DMA writes to PSRAM bypassing the CPU cache.
        // UNALIGNED allows the range to cross cache-line boundaries.
        esp_cache_msync(fb->buf, fb->len,
                        ESP_CACHE_MSYNC_FLAG_INVALIDATE | ESP_CACHE_MSYNC_FLAG_UNALIGNED);

        ICVConsumer* c = consumer_.load(std::memory_order_relaxed);
        if (c) c->onFrame(fb);

        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ── public API ────────────────────────────────────────────────────────────────

bool CameraService::begin() {
    constexpr size_t STACK_WORDS = (48 * 1024) / sizeof(StackType_t);
    const BaseType_t ok = xTaskCreatePinnedToCoreWithCaps(
        &CameraService::taskThunk, "cam",
        STACK_WORDS, this, 1, &th_, 0,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ok != pdPASS) {
        Serial.println("[Cam] task create FAILED");
        th_ = nullptr;
        return false;
    }
    return true;
}

void CameraService::setConsumer(ICVConsumer* next) {
    CameraMode newMode = next ? next->cameraMode() : currentMode_;

    if (newMode != currentMode_) {
        // Mode change: null consumer first so no frames are dispatched during
        // the camera reinit cycle.
        ICVConsumer* prev = consumer_.exchange(nullptr, std::memory_order_acq_rel);

        // Minimal pause: signal the camera task to stop, then wait for it to
        // exit esp_camera_fb_get() so we can safely call esp_camera_deinit().
        // DO NOT call suspend() here — suspend() stops XCLK (ledc_stop) and
        // sends COM7_SRST, which leaves the OV2640 in a mid-reset state.
        // When esp_camera_init() restarts XCLK from that state the OV2640
        // fails SCCB configuration and never outputs frames.
        requestSleep();
        for (int i = 0; i < 500; i++) {
            if (idle_) break;
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        if (prev) prev->onDeactivate();
        currentMode_ = newMode;

        // Stop DMA only — XCLK keeps running so OV2640 stays warm and
        // SCCB-ready for the new-mode init that follows immediately.
        esp_camera_deinit();
        cameraInited_ = initCamera_(newMode);
        if (!cameraInited_) Serial.println("[Cam] setConsumer: reinit failed!");

        consumer_.store(next, std::memory_order_release);
        if (next) next->onActivate();
        wakeUp();                       // let the capture loop resume
    } else {
        // Same mode: O(1) pointer swap, no hardware change needed.
        ICVConsumer* prev = consumer_.exchange(next, std::memory_order_acq_rel);
        if (prev) prev->onDeactivate();
        if (next) next->onActivate();
    }
}

bool CameraService::suspend() {
    requestSleep();
    // esp_camera_fb_get() has a 4-second internal timeout (FB_GET_TIMEOUT).
    // Wait at least 5 s for the capture task to finish any in-progress onFrame()
    // call and exit fb_get() before we touch hardware.  Calling esp_camera_deinit()
    // while the task is blocked inside cam_take() would vQueueDelete a live queue (UB).
    bool taskIdle = false;
    for (int i = 0; i < 500; i++) {
        if (idle_) { taskIdle = true; break; }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!taskIdle && cameraInited_) {
        Serial.println("[Cam] suspend: task did not go idle -- aborting deinit");
        return false;
    }

    sensor_t* s = esp_camera_sensor_get();
    if (s) {
        // COM7_SRST resets ALL OV2640 registers to POR defaults in one XCLK cycle --
        // analog blocks enter minimum-power state.  Stopping XCLK immediately after
        // prevents the boot-ROM reload sequence from re-enabling those blocks.
        Serial.printf("[Cam] OV2640 (PID=0x%02X) COM7_SRST -> XCLK off\n", s->id.PID);
        s->set_reg(s, 0x112, 0xFF, 0x80); // Sensor bank (bit8=1), COM7 = COM7_SRST
        ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
    }
    esp_camera_deinit();
    cameraInited_ = false;

    // Drive XCLK LOW (not floating) so OV2640 sees a definite no-clock state.
    // Data/sync pins reset to high-Z input.
    gpio_reset_pin(GPIO_NUM_10);
    gpio_set_direction(GPIO_NUM_10, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_10, 0);

    // Leave data/sync/SCCB GPIO-matrix routing intact — esp_camera_deinit()
    // stops the DMA but does NOT clear the GPIO matrix connections.  If we
    // call gpio_reset_pin() here those connections are severed and
    // esp_camera_init() cannot re-establish them reliably, causing fb_get()
    // to block indefinitely after a mode switch.
    Serial.println("[Cam] suspended, XCLK LOW");
    return true;
}

bool CameraService::resume() {
    Serial.println("[Cam] resume called");
    const bool ok = initCamera_(currentMode_);
    if (ok) {
        cameraInited_ = true;
        wakeUp();
    }
    return ok;
}
