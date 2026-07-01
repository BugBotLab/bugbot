// BoardPowerLib.h
// System power rail control: drives the peripheral enable pin, reads USB presence,
// and measures battery voltage via ADC with a resistor-divider correction factor.
#pragma once
#include <Arduino.h>

class BoardPowerLib {
public:
  struct Config {
    int   pinEnable = 43;
    int   pinUsbDetect = 1;
    int   pinBatAdc = 3;
    bool  enableActiveHigh = true;
    float adcRefVolts = 3.3f;
    int   adcMax = 4095;
    float batteryDividerRatio = 2.0f;
  };

  BoardPowerLib() = default;
  explicit BoardPowerLib(const Config& cfg) : _cfg(cfg) {}

  void configure(const Config& cfg) { _cfg = cfg; }

  void begin() {
    pinMode(_cfg.pinEnable, OUTPUT);
    setSystemEnabled(true);  // drive HIGH immediately — motor driver must never see an extended LOW after reset
    pinMode(_cfg.pinUsbDetect, INPUT);
  }

  void setSystemEnabled(bool en) {
    digitalWrite(_cfg.pinEnable,
                 ((_cfg.enableActiveHigh ? en : !en) ? HIGH : LOW));
    _systemEnabled = en;
  }

  bool systemEnabled() const {
    return _systemEnabled;
  }

  bool usbPresent() const {
    return digitalRead(_cfg.pinUsbDetect) == HIGH;
  }

float readBatteryVolts() const {
  int raw = analogRead(_cfg.pinBatAdc);
  float vPin = (raw / float(_cfg.adcMax)) * _cfg.adcRefVolts;
  return (vPin * _cfg.batteryDividerRatio) + 0.18f;
}

private:
  Config _cfg;
  bool _systemEnabled = false;
};
