// TinyMLService.cpp — TensorFlow Lite Micro inference on QQVGA frames.
//
// Requires: espressif/esp-tflite-micro added to idf_component.yml
//           Run `idf.py update-dependencies` to fetch the component.
#include "TinyMLService.h"
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <string.h>
#include <LittleFS.h>

// TFLite Micro headers — only included in the .cpp so the header stays light.
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

// Register a broad set of common ops.  Add more here if your model uses ops
// that fail with "Did not find kernel for op" in the serial log.
static tflite::MicroMutableOpResolver<TinyMLService::kNumOps>* buildResolver(void* mem) {
    using R = tflite::MicroMutableOpResolver<TinyMLService::kNumOps>;
    R* r = new (mem) R();
    r->AddConv2D();
    r->AddDepthwiseConv2D();
    r->AddFullyConnected();
    r->AddReshape();
    r->AddSoftmax();
    r->AddMaxPool2D();
    r->AddAveragePool2D();
    r->AddQuantize();
    r->AddDequantize();
    r->AddAdd();
    r->AddMul();
    r->AddRelu();
    r->AddRelu6();
    r->AddPad();
    r->AddMean();
    r->AddLogistic();
    return r;
}

// ── lifecycle ─────────────────────────────────────────────────────────────────

bool TinyMLService::begin() {
    frameMux_ = xSemaphoreCreateMutex();
    if (!frameMux_) {
        Serial.println("[TinyML] semaphore create failed");
        return false;
    }
    return true;
}

bool TinyMLService::loadModel(const char* path) {
    if (!LittleFS.begin(false)) {
        Serial.println("[TinyML] LittleFS mount failed");
        return false;
    }
    File f = LittleFS.open(path, "r");
    if (!f) {
        Serial.printf("[TinyML] model not found: %s\n", path);
        return false;
    }
    const size_t len = f.size();
    if (len == 0 || len > 4 * 1024 * 1024) {
        Serial.printf("[TinyML] model size out of range: %u bytes\n", (unsigned)len);
        f.close();
        return false;
    }

    uint8_t* buf = (uint8_t*)heap_caps_malloc(len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        Serial.println("[TinyML] PSRAM alloc failed for model");
        f.close();
        return false;
    }

    if (f.read(buf, len) != (int)len) {
        Serial.println("[TinyML] model read error");
        heap_caps_free(buf);
        f.close();
        return false;
    }
    f.close();

    heap_caps_free(modelData_);
    modelData_ = buf;
    modelLen_  = len;
    Serial.printf("[TinyML] model loaded: %s (%u bytes)\n", path, (unsigned)len);
    return true;
}

void TinyMLService::onActivate() {
    if (!modelData_) {
        Serial.println("[TinyML] no model loaded — call loadModel() first");
        return;
    }

    // Allocate PSRAM buffers
    arena_    = (uint8_t*)heap_caps_malloc(kArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    frameBuf_ = (uint8_t*)heap_caps_malloc(kW * kH,   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    resolverMem_ = heap_caps_malloc(
        sizeof(tflite::MicroMutableOpResolver<kNumOps>), MALLOC_CAP_8BIT);
    interpMem_   = heap_caps_malloc(sizeof(tflite::MicroInterpreter), MALLOC_CAP_8BIT);

    if (!arena_ || !frameBuf_ || !resolverMem_ || !interpMem_) {
        Serial.println("[TinyML] alloc failed");
        return;
    }

    const tflite::Model* model = tflite::GetModel(modelData_);
    if (!model) {
        Serial.println("[TinyML] GetModel failed — invalid flatbuffer");
        return;
    }

    auto* resolver   = buildResolver(resolverMem_);
    auto* interpreter = new (interpMem_) tflite::MicroInterpreter(
        model, *resolver, arena_, kArenaSize);

    if (interpreter->AllocateTensors() != kTfLiteOk) {
        Serial.println("[TinyML] AllocateTensors failed");
        return;
    }

    TfLiteTensor* in = interpreter->input(0);
    Serial.printf("[TinyML] input tensor: type=%d dims=[", in->type);
    for (int i = 0; i < in->dims->size; i++)
        Serial.printf("%s%d", i ? "," : "", in->dims->data[i]);
    Serial.printf("] bytes=%u\n", (unsigned)in->bytes);

    inferActive_ = true;
    const BaseType_t ok = xTaskCreatePinnedToCoreWithCaps(
        &TinyMLService::inferTaskThunk, "tinyml",
        (32 * 1024) / sizeof(StackType_t), this, 1, &inferHandle_, 1,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ok != pdPASS) {
        Serial.println("[TinyML] inference task create failed");
        inferActive_ = false;
    } else {
        Serial.println("[TinyML] inference task started");
    }
}

void TinyMLService::onDeactivate() {
    inferActive_ = false;
    if (inferHandle_) {
        // Wake the inference task so it can exit cleanly
        xTaskNotifyGive(inferHandle_);
        vTaskDelay(pdMS_TO_TICKS(100));
        vTaskDelete(inferHandle_);
        inferHandle_ = nullptr;
    }

    if (interpMem_) {
        reinterpret_cast<tflite::MicroInterpreter*>(interpMem_)->~MicroInterpreter();
        heap_caps_free(interpMem_); interpMem_ = nullptr;
    }
    if (resolverMem_) {
        using R = tflite::MicroMutableOpResolver<kNumOps>;
        reinterpret_cast<R*>(resolverMem_)->~MicroMutableOpResolver();
        heap_caps_free(resolverMem_); resolverMem_ = nullptr;
    }
    heap_caps_free(arena_);    arena_    = nullptr;
    heap_caps_free(frameBuf_); frameBuf_ = nullptr;
    frameReady_ = false;
    Serial.println("[TinyML] deactivated");
}

// ── per-frame capture (called from camera task) ───────────────────────────────

void TinyMLService::onFrame(camera_fb_t* fb) {
    if (!frameBuf_ || !inferHandle_) return;
    // Non-blocking try: if inference task is still running, skip this frame.
    if (xSemaphoreTake(frameMux_, 0) == pdTRUE) {
        memcpy(frameBuf_, fb->buf, (size_t)(kW * kH));
        frameReady_ = true;
        xSemaphoreGive(frameMux_);
        xTaskNotifyGive(inferHandle_);
    }
}

// ── inference task (Core 1, PSRAM stack) ─────────────────────────────────────

void TinyMLService::inferTask() {
    while (inferActive_) {
        // Block until a new frame is available (or deactivate wakes us)
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));

        if (!inferActive_) break;
        if (!interpMem_)   continue;

        auto* interp = reinterpret_cast<tflite::MicroInterpreter*>(interpMem_);
        TfLiteTensor* in = interp->input(0);
        if (!in || !in->data.uint8) continue;

        // Copy frame to input tensor under mutex (fast, <1 ms for 19 KB)
        xSemaphoreTake(frameMux_, portMAX_DELAY);
        if (!frameReady_) { xSemaphoreGive(frameMux_); continue; }
        const size_t copyBytes = (in->bytes < (size_t)(kW * kH))
                                 ? in->bytes : (size_t)(kW * kH);
        memcpy(in->data.uint8, frameBuf_, copyBytes);
        frameReady_ = false;
        xSemaphoreGive(frameMux_);

        // Run inference — release mutex first so onFrame() can write next frame
        if (interp->Invoke() != kTfLiteOk) {
            Serial.println("[TinyML] Invoke failed");
            continue;
        }

        TfLiteTensor* out = interp->output(0);
        if (!out || !out->data.f) continue;

        const int nClasses = out->dims->data[out->dims->size - 1];
        if (resultCb_) resultCb_(out->data.f, nClasses);
    }
    vTaskDelete(nullptr);
}
