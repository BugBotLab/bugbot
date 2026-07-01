// CameraService.h
// Owns the OV2640 camera hardware: init, power-down/up, and frame capture loop.
// Dispatches every captured frame to a single ICVConsumer via an atomic pointer
// swap — switching consumers is O(1) and safe to call from any task.
//
// Dynamic mode switching: if the new consumer requests a different camera
// format/resolution than the current config, setConsumer() automatically
// suspends the camera, reinitialises the OV2640, and resumes.  Callers may
// block for up to 5 s during the reinit on the first switch.
//
// Frame loop runs on Core 0 (same core as AprilTag, free of WiFi stack).
#pragma once
#include <Arduino.h>
#include <atomic>
#include "../core/ICVConsumer.h"
#include "../util/TaskUtil.hpp"

extern volatile bool g_cameraStreaming; // set by CameraAPI when MJPEG stream active

class CameraService {
public:
    // Start the frame-capture task.  Returns true on camera init success.
    bool begin();

    // Set (or clear) the active CV consumer.  If the new consumer's cameraMode()
    // differs from the current OV2640 config, the camera is power-cycled and
    // reinitialised with the new settings before onActivate() is called.
    // Calls onDeactivate() on the previous consumer and onActivate() on the new one.
    // Pass nullptr to run the camera with no consumer (idle capture loop).
    void setConsumer(ICVConsumer* next);

    ICVConsumer* consumer() const { return consumer_.load(std::memory_order_relaxed); }

    // Power management — safe to call from any task.
    // suspend(): power-down sequence (COM7_SRST + XCLK off + deinit + GPIO reset).
    //            Blocks until the capture task is idle (up to 5 s) before touching
    //            hardware so we never call esp_camera_deinit() on a live queue.
    // resume():  re-init camera with the current mode and restart the capture loop.
    bool suspend();
    bool resume();

    void requestSleep() { sleepReq_ = true; }
    void wakeUp()       { sleepReq_ = false; }
    bool isIdle()       const { return idle_; }
    bool isInited()     const { return cameraInited_; }

    const CameraMode& currentMode() const { return currentMode_; }

private:
    static void taskThunk(void* arg) { static_cast<CameraService*>(arg)->run(); }
    void run();
    bool initCamera_(const CameraMode& mode);
    static void frameSizeDims(framesize_t fs, int& w, int& h);

    TaskHandle_t              th_           = nullptr;
    std::atomic<ICVConsumer*> consumer_     { nullptr };
    volatile bool             sleepReq_     = false;
    volatile bool             idle_         = false;
    volatile bool             cameraInited_ = false;
    CameraMode                currentMode_  {};      // current OV2640 configuration
    volatile int              expW_         = 160;   // expected frame width (from initCamera_)
    volatile int              expH_         = 120;   // expected frame height
};
