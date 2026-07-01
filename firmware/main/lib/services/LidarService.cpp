#include "LidarService.h"
#include "../core/Transforms.hpp"
#include "../core/BusLocks.hpp"

bool LidarService::start(uint8_t row, uint8_t hz){
  row_ = row; hz_ = hz;
  return CreateTaskPinnedPSRAM("lidar", &LidarService::taskThunk, 16, 2, &th_, 1, this);
}

void LidarService::run(){
  vTaskDelay(pdMS_TO_TICKS(300));

  bool ok = false;
  if (I2C_LOCK(pdMS_TO_TICKS(200))) {
    ok = lidar_.begin(Wire);
    if (ok) {
      lidar_.setRow(row_);
      lidar_.setRangingFrequency(hz_);
      auto cal = lidar_.calibration();
      cal.fov_deg     = 60.0f;
      cal.x_off_mm    = 40.0f;    // sensor is 40mm ahead of robot centre
      cal.y_off_mm    = 0.0f;
      cal.z_off_mm    = 50.0f;    // sensor height above the floor (mm) — tune to match your mount
      cal.yaw_off_deg = 0.0f;     // sensor faces forward (same direction as robot)
      cal.flip_lr     = false;
      lidar_.setCalibration(cal);
      lidar_.setAzimuthStraightening(0.5f, 0.0f);
      lidar_.useAzimuthLUT(false);
      lidar_.start();
      sensorStarted_ = true;
    }
    I2C_UNLOCK();
  }

  { portENTER_CRITICAL(&mux_); ok_ = ok; portEXIT_CRITICAL(&mux_); }

  uint16_t s[4];
  uint16_t g[64];
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS((hz_ > 0) ? (1000u / (uint32_t)hz_) : 50u);

  for (;;) {
    if (sleepRequested_) {
      if (sensorStarted_) {
        if (I2C_LOCK(pdMS_TO_TICKS(100))) {
          lidar_.stop();
          I2C_UNLOCK();
        }
        sensorStarted_ = false;
      }

      portENTER_CRITICAL(&mux_);
      ok_ = false;
      for (int i=0;i<4;i++) mm_[i]=0;
      portEXIT_CRITICAL(&mux_);

      idle_ = true;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    idle_ = false;

    bool got = false;
    if (I2C_LOCK(pdMS_TO_TICKS(60))) {
      got = lidar_.getGrid(g, 50);
      I2C_UNLOCK();
    }

    if (got) {
      // Sample 4 evenly-spaced columns from the selected row (cols 0, 2, 4, 6)
      const int row = lidar_.row();
      for (int i = 0; i < 4; i++) s[i] = g[row * 8 + i * 2];

      portENTER_CRITICAL(&mux_);
      for (int i = 0; i < 4;  i++) mm_[i]   = s[i];
      for (int i = 0; i < 64; i++) grid_[i] = g[i];
      ok_ = true;
      gen_++;
      portEXIT_CRITICAL(&mux_);
    }

    vTaskDelayUntil(&last, period);
  }
}

void LidarService::stopSensor(){
  if (I2C_LOCK(pdMS_TO_TICKS(100))) {
    lidar_.stop();
    I2C_UNLOCK();
  }
  sensorStarted_ = false;
  portENTER_CRITICAL(&mux_);
  ok_ = false;
  for (int i=0;i<4;i++) mm_[i]=0;
  portEXIT_CRITICAL(&mux_);
}
