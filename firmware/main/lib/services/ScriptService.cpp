#include "ScriptService.h"
#include "EspNowService.h"
#include "bugbot_shims.h"
#include "pocketpy.h"

extern "C" void bugbot_module_init(void);

#include <esp_heap_caps.h>
#include <string.h>
#include <stdio.h>

/* Single active ScriptService — used by the static pocketpy print callback. */
static ScriptService* s_current = nullptr;

// pocketpy requires the FreeRTOS task stack to be in internal DRAM.
// Xtensa window-overflow exceptions fire in ISR context and cannot tolerate PSRAM.
static StackType_t  s_task_stack[32 * 1024 / sizeof(StackType_t)]
    __attribute__((section(".dram0.bss")));
static StaticTask_t s_task_tcb
    __attribute__((section(".dram0.bss")));

// ── onBegin ────────────────────────────────────────────────────────────────────
void ScriptService::onBegin(const uint8_t* ownerMac, uint32_t totalSize, uint8_t scriptId) {
  // Signal any running script to abort — non-blocking.
  // onBegin() runs inside the ESP-NOW WiFi callback; blocking here (vTaskDelay,
  // semaphore waits, etc.) stalls the WiFi task for the duration, causing the
  // CHUNK and RUN packets that arrive immediately after BEGIN to be dropped.
  stopReq_ = true;

  memcpy(ownerMac_, ownerMac, 6);
  scriptId_  = scriptId;
  totalSize_ = (totalSize > kMaxScript) ? kMaxScript : totalSize;
  bytesIn_   = 0;
  nextChunk_ = 0;

  // (Re)allocate PSRAM buffer.
  if (buf_ && bufCap_ < totalSize_) {
    heap_caps_free(buf_);
    buf_ = nullptr;
  }
  if (!buf_) {
    buf_    = (uint8_t*)heap_caps_malloc(totalSize_ + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    bufCap_ = buf_ ? totalSize_ : 0;
  }
  if (!buf_) {
    Serial.printf("[SCRIPT] PSRAM alloc failed (%lu bytes)\n", (unsigned long)totalSize_);
    return;
  }
  Serial.printf("[SCRIPT] BEGIN size=%lu id=%u\n", (unsigned long)totalSize_, scriptId_);

  // Create the semaphore and persistent task exactly once.
  if (!runSem_) {
    runSem_ = xSemaphoreCreateBinary();
  }
  if (!scriptTask_) {
    scriptTask_ = xTaskCreateStaticPinnedToCore(
      taskThunk_, "ScriptTask",
      kTaskStack / sizeof(StackType_t), this,
      kTaskPrio, s_task_stack, &s_task_tcb,
      0  // Core 0
    );
    Serial.println("[SCRIPT] persistent task created");
  }

  // Drain any leftover semaphore token from a previous run.
  if (runSem_) xSemaphoreTake(runSem_, 0);
}

// ── onChunk ────────────────────────────────────────────────────────────────────
void ScriptService::onChunk(uint16_t chunkIdx, const uint8_t* data, uint8_t dataLen) {
  if (!buf_ || !data || dataLen == 0) return;

  if (chunkIdx != nextChunk_) {
    Serial.printf("[SCRIPT] chunk %u unexpected (want %u) — drop\n", chunkIdx, nextChunk_);
    return;
  }

  uint32_t offset = (uint32_t)chunkIdx * 220;
  if (offset + dataLen > totalSize_) {
    dataLen = (uint8_t)(totalSize_ - offset);
  }
  memcpy(buf_ + offset, data, dataLen);
  bytesIn_ += dataLen;
  nextChunk_++;
}

// ── onRun ──────────────────────────────────────────────────────────────────────
void ScriptService::onRun(const uint8_t* ownerMac, uint8_t scriptId) {
  (void)scriptId;
  if (!buf_ || bytesIn_ == 0) {
    Serial.println("[SCRIPT] RUN with no script loaded");
    sendDone(1);
    return;
  }
  if (!scriptTask_ || !runSem_) {
    Serial.println("[SCRIPT] RUN but task/semaphore not ready");
    sendDone(1);
    return;
  }

  memcpy(ownerMac_, ownerMac, 6);
  buf_[bytesIn_] = '\0';

  Serial.printf("[SCRIPT] RUN %lu bytes\n", (unsigned long)bytesIn_);
  stopReq_ = false;
  xSemaphoreGive(runSem_);  // wake the persistent task
}

// ── onStop ─────────────────────────────────────────────────────────────────────
void ScriptService::onStop() {
  // Non-blocking: just set the flag. The ScriptTask observes it via
  // bugbot_shim_check_stop() and at the top of each loop iteration.
  // Never block here — this is called from the ESP-NOW WiFi callback.
  stopReq_ = true;
}

// ── executeScript_ ────────────────────────────────────────────────────────────
// Runs forever in the persistent ScriptTask.
// py_initialize() is called once here — pk_current_vm is set for THIS task's
// TLS and is preserved across all runs because the task never exits.
void ScriptService::executeScript_() {
  Serial.println("[SCRIPT] task ready");
  py_initialize();

  while (true) {
    xSemaphoreTake(runSem_, portMAX_DELAY);

    if (stopReq_) continue;

    running_ = true;
    s_current = this;
    bugbot_shim_set_stop_ptr(&stopReq_);

    // Set callbacks after every py_resetvm() call (resetvm wipes the callback table).
    py_Callbacks* cb = py_callbacks();
    cb->print = [](const char* s) {
      if (!s_current) return;
      s_current->printBuf_.append(s);
      size_t pos;
      while ((pos = s_current->printBuf_.find('\n')) != std::string::npos) {
        std::string line = s_current->printBuf_.substr(0, pos + 1);
        s_current->sendLog(line.c_str(), line.size());
        s_current->printBuf_.erase(0, pos + 1);
      }
      if (s_current->printBuf_.size() >= 230) {
        s_current->sendLog(s_current->printBuf_.c_str(), s_current->printBuf_.size());
        s_current->printBuf_.clear();
      }
    };
    cb->flush = [](){ };

    bugbot_module_init();

    // Notify host that the new script is about to run. This lets the host
    // distinguish "ScriptDone from old script cleanup" (arrives before ScriptStarted)
    // from "ScriptDone from the new script" (arrives after ScriptStarted).
    sendScriptPkt_(ENT_SCRIPT_STARTED, nullptr, 0);

    bool ok = py_exec((const char*)buf_, "<student_script>", EXEC_MODE, NULL);
    Serial.printf("[SCRIPT] py_exec ok=%d\n", (int)ok);
    if (!ok) py_printexc();

    if (!printBuf_.empty()) {
      sendLog(printBuf_.c_str(), printBuf_.size());
      printBuf_.clear();
    }

    py_resetvm();

    bugbot_shim_stop();
    bugbot_shim_set_stop_ptr(nullptr);
    s_current = nullptr;
    running_ = false;

    uint8_t code = ok ? 0 : (stopReq_ ? 130 : 1);
    sendDone(code);
  }
}

// ── sendLog / sendDone ─────────────────────────────────────────────────────────
void ScriptService::sendLog(const char* line, size_t len) {
  if (!espnow_ || len == 0) return;
  if (len > 239) len = 239;
  sendScriptPkt_(ENT_SCRIPT_LOG, (const uint8_t*)line, (uint8_t)len);
}

void ScriptService::sendDone(uint8_t exitCode) {
  if (!espnow_) return;
  sendScriptPkt_(ENT_SCRIPT_DONE, &exitCode, 1);
}

// ── sendScriptPkt_ ─────────────────────────────────────────────────────────────
void ScriptService::sendScriptPkt_(uint8_t pktType, const uint8_t* payload, uint8_t plen) {
  if (!espnow_) return;
  espnow_->sendScriptPacket_(ownerMac_, pktType, payload, plen);
}
