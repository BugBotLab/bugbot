// OdomService.h
// FreeRTOS task that samples the BMI270 at a fixed rate, integrates yaw into
// Odometry2D, and publishes Pose2D to PoseBus. PoseService calls correctYawRad()
// when an AprilTag absolute fix is accepted to re-anchor the odometry heading.
#pragma once
#include <Arduino.h>
#include "../drivers/BMI270Driver.h"
#include "../core/PoseBus.h"
#include "../util/TaskUtil.hpp"

class OdomService {
public:
  bool start(PoseBus& pose, int hz = 250);
  bool prepareForSleep();

  // Shift the IMU yaw offset so subsequent readings report target_yaw_rad.
  // Thread-safe to call from PoseService when an AprilTag absolute fix is accepted.
  void correctYawRad(float target_yaw_rad);
  bool  recalibrateBias()    { return imuReady_ ? imu_.recalibrateBias() : false; }
  float getGyroBiasDps()     const { return imuReady_ ? imu_.getBiasZDps()       : 0.0f; }
  float getGyroFiltDps()     const { return imuReady_ ? imu_.getFiltZDps()       : 0.0f; }
  // Returns horizontal accel magnitude in g-units; 9.9 when IMU not ready (disables ZUPT).
  float getAccelHorizMagG()  const { return imuReady_ ? imu_.getAccelHorizMagG() : 9.9f; }

  void requestSleep() { sleepRequested_ = true; }
  bool isIdle() const { return idle_; }

  // Backward-compatible alias; now cooperative rather than force-suspending mid-transaction.
  void suspend() { requestSleep(); }

private:
  static void taskThunk(void* arg) { static_cast<OdomService*>(arg)->run(); }
  void run();

  PoseBus*     pose_ = nullptr;
  int          hz_   = 250;
  TaskHandle_t th_   = nullptr;
  BMI270Driver imu_;
  volatile bool imuReady_ = false;
  volatile bool sleepRequested_ = false;
  volatile bool idle_ = false;
};
