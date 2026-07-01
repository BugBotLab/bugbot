// ScriptService.h
// Receives Python scripts over ESP-NOW (ECS_SCRIPT_BEGIN/CHUNK/RUN/STOP),
// reassembles them in a PSRAM buffer, then executes them in a persistent
// FreeRTOS task (created once on first BEGIN, never deleted).
//
// The persistent task strategy means py_initialize() runs exactly once,
// keeping pk_current_vm valid in that task's TLS for all subsequent runs.
// py_resetvm() cleans the VM between runs without touching TLS.
//
// Thread safety: attach() and onXxx() are called from EspNowService::tick()
// (single-threaded); sendLog() and sendDone() are called from the script task
// (a separate FreeRTOS task). esp_now_send() is internally thread-safe.
#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <string>

class EspNowService;

class ScriptService {
public:
  // Wire EspNowService in before any callbacks arrive.
  void attach(EspNowService* espnow) { espnow_ = espnow; }

  // Called from EspNowService::onCommand_ when the matching ECS_SCRIPT_* sub-type is seen.
  // ownerMac is the 6-byte ESP-NOW source MAC of the claimed host.
  void onBegin(const uint8_t* ownerMac, uint32_t totalSize, uint8_t scriptId);
  void onChunk(uint16_t chunkIdx, const uint8_t* data, uint8_t dataLen);
  void onRun(const uint8_t* ownerMac, uint8_t scriptId);
  void onStop();

  // Called by the script task or the pocketpy stdout hook to push output.
  // These are thread-safe (call esp_now_send which is internally queued).
  void sendLog(const char* line, size_t len);
  void sendDone(uint8_t exitCode);

private:
  static void taskThunk_(void* arg) { static_cast<ScriptService*>(arg)->executeScript_(); }
  void executeScript_();

  void sendScriptPkt_(uint8_t pktType, const uint8_t* payload, uint8_t plen);

  EspNowService* espnow_    = nullptr;
  uint8_t  ownerMac_[6]     = {};

  // Script buffer in PSRAM.
  uint8_t* buf_             = nullptr;
  uint32_t bufCap_          = 0;
  uint32_t totalSize_       = 0;
  uint32_t bytesIn_         = 0;
  uint8_t  scriptId_        = 0;

  // Persistent task: created once on first onBegin(), stays alive forever.
  // onRun() gives runSem_ to wake it; onStop() waits on running_.
  TaskHandle_t      scriptTask_ = nullptr;
  SemaphoreHandle_t runSem_     = nullptr;
  volatile bool     running_    = false;
  volatile bool     stopReq_    = false;

  uint16_t nextChunk_ = 0;

  std::string printBuf_;

  static constexpr uint32_t kMaxScript  = 64 * 1024;
  static constexpr uint32_t kTaskStack  = 32 * 1024;
  static constexpr int      kTaskPrio   = 2;
};
