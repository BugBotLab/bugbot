// TinyMLService.h
// TensorFlow Lite Micro inference as an ICVConsumer.
//
// Architecture:
//   Camera task → onFrame() → copies 160×120 frame → signals inference task
//   Inference task → preprocesses → TFLite Invoke() → result callback
//
// onFrame() returns immediately (never blocks camera task), so the capture
// loop continues at full speed while inference runs in parallel.  If inference
// is still running when the next frame arrives, that frame is dropped (newest-
// frame-wins semantics).
//
// Dependencies:
//   idf_component.yml must include:  espressif/esp-tflite-micro: ">=1.0.0"
//   Run `idf.py update-dependencies` after adding the component.
//
// Model format:
//   /models/model.tflite on LittleFS (loaded by loadModel()).
//   Input tensor must be UINT8, shape [1, H, W, 1] or [1, H*W].
//   Output tensor must be FLOAT32 with one score per class.
#pragma once
#include <Arduino.h>
#include <functional>
#include "../core/ICVConsumer.h"

// Forward-declare TFLite types so the header compiles without pulling in all
// of TFLite Micro (which adds ~2 s to incremental builds).
namespace tflite {
    class MicroInterpreter;
    class MicroOpResolver;
    template<unsigned int> class MicroMutableOpResolver;
}

class TinyMLService : public ICVConsumer {
public:
    static constexpr int    kW         = 160;
    static constexpr int    kH         = 120;
    static constexpr size_t kArenaSize = 512 * 1024; // 512 KB tensor arena in PSRAM

    // Invoked from the inference task after each successful Invoke().
    // scores[0..nClasses-1] are FLOAT32 softmax probabilities.
    using ResultCb = std::function<void(const float* scores, int nClasses)>;
    void setCallback(ResultCb cb) { resultCb_ = std::move(cb); }

    bool begin();

    // Load a .tflite model from LittleFS.  Call after begin() and before
    // CameraService::setConsumer() selects this service.
    // Returns false if the file is missing or the model is invalid.
    bool loadModel(const char* path = "/models/model.tflite");

    bool hasModel() const { return modelData_ != nullptr; }

    // ICVConsumer interface
    void onFrame(camera_fb_t* fb) override;
    void onActivate()             override;
    void onDeactivate()           override;

    static constexpr unsigned int kNumOps = 16;

private:
    static void inferTaskThunk(void* arg) { static_cast<TinyMLService*>(arg)->inferTask(); }
    void inferTask();

    ResultCb resultCb_{};

    // Model storage (PSRAM) — kept alive as long as the interpreter uses it.
    uint8_t* modelData_ = nullptr;
    size_t   modelLen_  = 0;

    // Tensor arena (PSRAM)
    uint8_t* arena_ = nullptr;

    // Frame double-buffer: onFrame() writes, inferTask() reads.
    uint8_t*          frameBuf_ = nullptr;
    SemaphoreHandle_t frameMux_ = nullptr; // protects frameBuf_
    volatile bool     frameReady_ = false;

    // Inference task
    TaskHandle_t inferHandle_ = nullptr;
    volatile bool inferActive_ = false;

    // TFLite objects (heap in DRAM — interpreter is not PSRAM-safe on all targets)
    void* resolverMem_  = nullptr;
    void* interpMem_    = nullptr;
};
