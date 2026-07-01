// LedLib.h
// Simple GPIO digital LED driver (on/off). For the WS2812 RGB LED use RgbLedLib.h.
#pragma once
#include <Arduino.h>

class LedLib {
public:
  void begin(int pin) {
    _pin = pin;
    pinMode(_pin, OUTPUT);
    off();
  }

  void on() {
    digitalWrite(_pin, HIGH);
  }

  void off() {
    digitalWrite(_pin, LOW);
  }

private:
  int _pin = -1;
};