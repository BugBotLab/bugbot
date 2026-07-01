// BuzzerLib.h
// PWM buzzer driver using the ESP32 ledc peripheral. Supports runtime frequency
// and duty-cycle control; call off() to silence.
#pragma once
#include <Arduino.h>

class BuzzerLib {
public:
  void begin(int pin, uint8_t resolutionBits = 10) {
    _pin = pin;
    _resolutionBits = resolutionBits;

    // Core 3.x API: configure PWM directly on the pin
    const bool ok = ledcAttach(_pin, 2000, _resolutionBits);
    _attached = ok;
    off();
  }

  void tone(uint32_t freqHz, float duty01 = 0.5f) {
    if (!_attached) return;

    if (freqHz == 0) {
      off();
      return;
    }

    // ledcWriteTone removed in arduino-esp32 3.x; use ledcChangeFrequency instead
    const uint32_t actual = ledcChangeFrequency(_pin, freqHz, _resolutionBits);
    if (actual == 0) {
      off();
      return;
    }

    const uint32_t maxDuty = (1u << _resolutionBits) - 1u;
    const uint32_t duty =
        (uint32_t)(constrain(duty01, 0.0f, 1.0f) * maxDuty);

    ledcWrite(_pin, duty);
    _isOn = true;
  }

  void off() {
    if (!_attached) return;
    ledcWrite(_pin, 0);
    _isOn = false;
  }

  bool isOn() const { return _isOn; }

private:
  int _pin = -1;
  uint8_t _resolutionBits = 10;
  bool _attached = false;
  bool _isOn = false;
};