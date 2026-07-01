// MotionService.h
// FreeRTOS task that translates drive commands (discrete DriveDir or continuous 3-axis
// vector) to PCA9685 motor PWM via MotionLib. Implements heading-hold PID using
// PoseBus yaw, slew-rate limiting, and auto power management.
#pragma once
#include <Arduino.h>
#include "../drivers/MotionLib.h"
#include "../drivers/BoardPowerLib.h"
#include "../core/PoseBus.h"
#include "../util/TaskUtil.hpp"
#include "../core/DriveDefs.hpp"
#include "../config/AppConfig.h"

class MotionService {
public:
  void configure(const MotionRuntimeConfig& cfg) { cfg_ = cfg; }
  bool start(PoseBus& pose, BoardPowerLib* power = nullptr);
  void requestSleep() { sleepRequested_ = true; }
  bool isIdle() const { return idle_; }
  void suspend() { requestSleep(); }
  void setCommand(DriveDir dir, float speed);
  void setCommandVec(float longitudinal, float lateral, float rotational);
  void getLastVecCmd(float& log, float& lat, float& rot) const {
    log = longitudinal_; lat = lateral_; rot = rotational_;
  }

  void configureAutoPower(bool enabled, uint32_t idleTimeoutMs, uint32_t powerOnSettleMs) {
    autoPowerEnabled_ = enabled;
    idleTimeoutMs_ = idleTimeoutMs;
    powerOnSettleMs_ = powerOnSettleMs;
  }
  // Call when the system power rail is restored externally (e.g. by loop() auto-power
  // management) so the motion task re-initialises the PCA9685.
  void signalPowerRestored() {
    needsReinit_ = true;
    if (th_) xTaskNotifyGive(th_);
  }
  bool wantsPeripheralPower(uint32_t nowMs) const;

private:
  static void taskThunk(void* arg){ static_cast<MotionService*>(arg)->run(); }
  void run();

  PoseBus*          pose_{nullptr};
  volatile float    longitudinal_{0.0f};
  volatile float    lateral_{0.0f};
  volatile float    rotational_{0.0f};
  TaskHandle_t      th_{nullptr};
  BoardPowerLib*    power_{nullptr};
  MotionRuntimeConfig cfg_{};

  bool   headingHoldActive_ = false;
  float  headingTargetDeg_ = 0.0f;
  volatile bool sleepRequested_ = false;
  volatile bool idle_ = false;
  bool autoPowerEnabled_ = false;
  uint32_t idleTimeoutMs_ = 1200;
  uint32_t powerOnSettleMs_ = 12;
  volatile uint32_t lastNonZeroCommandMs_ = 0;
  volatile bool needsReinit_ = false;
};
