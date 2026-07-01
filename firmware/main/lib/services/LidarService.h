// LidarService.h
// FreeRTOS task that polls LidarLib at a fixed rate and double-buffers the 8×8
// distance grid under a critical section so callers never read a partial update.
// Exposes getStrip() (selected 4-zone row) and getGrid() (full 64-zone grid).
#pragma once
#include <Arduino.h>
#include "../drivers/LidarLib.h"
#include "../util/TaskUtil.hpp"

class LidarService {
public:
  bool start(uint8_t row=1, uint8_t hz=15);  // 8×8 mode max = 15 Hz
  void requestSleep() { sleepRequested_ = true; }
  bool isIdle() const { return idle_; }
  void suspend() { requestSleep(); }
  void stopSensor();

  bool getStrip(uint16_t out[4]) const {
    portENTER_CRITICAL(&mux_);
    for (int i = 0; i < 4; i++) out[i] = mm_[i];
    bool ok = ok_;
    portEXIT_CRITICAL(&mux_);
    return ok;
  }
  // Full 8×8 grid (row-major, 64 values). Returns false if no valid data.
  bool getGrid(uint16_t out[64]) const {
    portENTER_CRITICAL(&mux_);
    for (int i = 0; i < 64; i++) out[i] = grid_[i];
    bool ok = ok_;
    portEXIT_CRITICAL(&mux_);
    return ok;
  }
  // Increments each time new sensor data arrives — lets callers detect fresh scans.
  uint32_t generation() const {
    portENTER_CRITICAL(&mux_); uint32_t g = gen_; portEXIT_CRITICAL(&mux_); return g;
  }
  uint8_t row() const { return row_; }
  uint8_t hz()  const { return hz_;  }
  const LidarLib& lib() const { return lidar_; }

private:
  static void taskThunk(void* arg){ static_cast<LidarService*>(arg)->run(); }
  void run();

  mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  uint16_t  mm_[4]    = {0};
  uint16_t  grid_[64] = {0};
  bool      ok_      = false;
  uint32_t  gen_     = 0;

  LidarLib  lidar_;
  uint8_t   row_ = 3;
  uint8_t   hz_  = 15;   // 8×8 mode max is 15 Hz
  TaskHandle_t th_ = nullptr;
  volatile bool sleepRequested_ = false;
  volatile bool idle_ = false;
  volatile bool sensorStarted_ = false;
};
