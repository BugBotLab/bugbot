#include "ActuatorService.h"
#include <stdlib.h>
#include <string.h>

static bool findNumberAfterKey_(const char* msg, const char* key, float& outVal) {
  if (!msg || !key) return false;

  const char* p = strstr(msg, key);
  if (!p) return false;

  p = strchr(p, ':');
  if (!p) return false;
  ++p;

  while (*p == ' ' || *p == '\t' || *p == '\"') ++p;

  char* endp = nullptr;
  outVal = strtof(p, &endp);
  return endp != p;
}

static bool findIntAfterKey_(const char* msg, const char* key, int& outVal) {
  if (!msg || !key) return false;

  const char* p = strstr(msg, key);
  if (!p) return false;

  p = strchr(p, ':');
  if (!p) return false;
  ++p;

  while (*p == ' ' || *p == '\t' || *p == '\"') ++p;

  char* endp = nullptr;
  long v = strtol(p, &endp, 10);
  if (endp == p) return false;
  outVal = (int)v;
  return true;
}

bool ActuatorService::begin(const RobotConfig& cfg, BoardPowerLib* power, bool autoPower, uint32_t idleTimeoutMs, uint32_t powerOnSettleMs) {
  power_ = power;
  autoPowerEnabled_ = autoPower;
  idleTimeoutMs_ = idleTimeoutMs;
  powerOnSettleMs_ = powerOnSettleMs;
  lastPeripheralUseMs_ = millis();

  servo_.begin(cfg.pins.servo1_pwm, cfg.pins.servo2_pwm);
  buzzer_.begin(cfg.pins.buzzer_pwm);
  led_.begin(cfg.pins.rgb_led_data);

  setServo1Deg(90.0f);
  setServo2Deg(90.0f);
  buzzerOff();
  ledOff();
  return true;
}


void ActuatorService::configureAutoPower(BoardPowerLib* power, bool enabled, uint32_t idleTimeoutMs, uint32_t powerOnSettleMs) {
  power_ = power;
  autoPowerEnabled_ = enabled;
  idleTimeoutMs_ = idleTimeoutMs;
  powerOnSettleMs_ = powerOnSettleMs;
}

void ActuatorService::ensurePeripheralPowerOn_() {
  if (!autoPowerEnabled_ || !power_) return;
  if (!power_->systemEnabled()) {
    power_->setSystemEnabled(true);
    delay(powerOnSettleMs_);
  }
}

void ActuatorService::markPeripheralUse_() {
  lastPeripheralUseMs_ = millis();
}

bool ActuatorService::wantsPeripheralPower(uint32_t nowMs) const {
  if (!autoPowerEnabled_) return true;
  if (buzzerFreqHz_ > 0) return true;
  return (nowMs - lastPeripheralUseMs_) < idleTimeoutMs_;
}

void ActuatorService::setServo1Deg(float deg) {
  ensurePeripheralPowerOn_();
  markPeripheralUse_();
  servo1Deg_ = constrain(deg, 0.0f, 180.0f);
  servo_.setServo1Deg(servo1Deg_);
}

void ActuatorService::setServo2Deg(float deg) {
  ensurePeripheralPowerOn_();
  markPeripheralUse_();
  servo2Deg_ = constrain(deg, 0.0f, 180.0f);
  servo_.setServo2Deg(servo2Deg_);
}

void ActuatorService::setBuzzerTone(uint32_t freqHz, float duty01) {
  ensurePeripheralPowerOn_();
  markPeripheralUse_();
  buzzerFreqHz_ = freqHz;
  buzzer_.tone(freqHz, duty01);
}

void ActuatorService::buzzerOff() {
  buzzerFreqHz_ = 0;
  buzzer_.off();
}

void ActuatorService::setLed(uint8_t r, uint8_t g, uint8_t b) {
  markPeripheralUse_();
  ledR_ = r;
  ledG_ = g;
  ledB_ = b;
  led_.set(r, g, b);
}

void ActuatorService::ledOff() {
  setLed(0, 0, 0);
}

void ActuatorService::detachServos() {
  servo_.detachAll();
}

void ActuatorService::allOff() {
  buzzerOff();
  ledOff();
  detachServos();
}

bool ActuatorService::handleWsText(const char* msg) {
  if (!msg) return false;

  bool handled = false;

  float servo1Deg;
  if (findNumberAfterKey_(msg, "\"servo1\"", servo1Deg)) {
    setServo1Deg(servo1Deg);
    handled = true;
  }

  float servo2Deg;
  if (findNumberAfterKey_(msg, "\"servo2\"", servo2Deg)) {
    setServo2Deg(servo2Deg);
    handled = true;
  }

  int buzzerHz;
  if (findIntAfterKey_(msg, "\"buzzer\"", buzzerHz)) {
    if (buzzerHz <= 0) buzzerOff();
    else setBuzzerTone((uint32_t)buzzerHz);
    handled = true;
  }

  int r, g, b;
  bool gotR = findIntAfterKey_(msg, "\"r\"", r);
  bool gotG = findIntAfterKey_(msg, "\"g\"", g);
  bool gotB = findIntAfterKey_(msg, "\"b\"", b);
  if (gotR && gotG && gotB) {
    if (wsLedWritesEnabled_) {
      setLed((uint8_t)constrain(r, 0, 255),
             (uint8_t)constrain(g, 0, 255),
             (uint8_t)constrain(b, 0, 255));
    }
    handled = true;
  }

  return handled;
}