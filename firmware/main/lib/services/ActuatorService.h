// ActuatorService.h
// Aggregates servo, buzzer, and RGB LED control into one service. Also manages
// peripheral auto-power gating and parses WebSocket JSON actuator commands.
#pragma once
#include <Arduino.h>
#include "../config/RobotConfig.h"
#include "../drivers/ServoLib.h"
#include "../drivers/BuzzerLib.h"
#include "../drivers/RgbLedLib.h"
#include "../drivers/BoardPowerLib.h"

class ActuatorService {
public:
  bool begin(const RobotConfig& cfg, BoardPowerLib* power = nullptr, bool autoPower = false, uint32_t idleTimeoutMs = 1200, uint32_t powerOnSettleMs = 12);

  void setServo1Deg(float deg);
  void setServo2Deg(float deg);

  void setBuzzerTone(uint32_t freqHz, float duty01 = 0.5f);
  void buzzerOff();

  void setLed(uint8_t r, uint8_t g, uint8_t b);
  void ledOff();

  void setWsLedWritesEnabled(bool en) { wsLedWritesEnabled_ = en; }
  bool wsLedWritesEnabled() const { return wsLedWritesEnabled_; }
  uint8_t ledR() const { return ledR_; }
  uint8_t ledG() const { return ledG_; }
  uint8_t ledB() const { return ledB_; }

  // DroidChime calls lockLed() to suppress the battery-pulse from the main loop
  // for the duration of a sound sequence.
  void lockLed(uint32_t durationMs) { ledLockUntilMs_ = millis() + durationMs; }
  bool ledLocked() const { return millis() < ledLockUntilMs_; }

  void detachServos();
  void allOff();

  void configureAutoPower(BoardPowerLib* power, bool enabled, uint32_t idleTimeoutMs, uint32_t powerOnSettleMs);
  bool wantsPeripheralPower(uint32_t nowMs) const;

  // Handles WS text frames such as:
  // {"servo1":90}
  // {"servo2":45}
  // {"buzzer":2000}
  // {"buzzer":0}
  // {"led":{"r":255,"g":0,"b":0}}
  bool handleWsText(const char* msg);

private:
  ServoLib  servo_;
  BuzzerLib buzzer_;
  RgbLedLib led_;

  float servo1Deg_ = 90.0f;
  float servo2Deg_ = 90.0f;
  uint32_t buzzerFreqHz_ = 0;
  uint8_t ledR_ = 0, ledG_ = 0, ledB_ = 0;
  bool wsLedWritesEnabled_ = true;
  volatile uint32_t ledLockUntilMs_ = 0;
  BoardPowerLib* power_ = nullptr;
  bool autoPowerEnabled_ = false;
  uint32_t idleTimeoutMs_ = 1200;
  uint32_t powerOnSettleMs_ = 12;
  uint32_t lastPeripheralUseMs_ = 0;

  void ensurePeripheralPowerOn_();
  void markPeripheralUse_();
};