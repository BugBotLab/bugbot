// RgbLedLib.h
// WS2812 NeoPixel RGB LED driver wrapping Adafruit_NeoPixel.
// Colour channels are corrected for the GRB byte-order of the physical strip.
#pragma once
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

class RgbLedLib {
public:
  void begin(int pin, uint16_t count = 1) {
    _pin = pin;
    _count = count;
    _strip = new Adafruit_NeoPixel(_count, _pin, NEO_GRB + NEO_KHZ800);
    _strip->begin();
    _strip->clear();
    _strip->show();
  }

  void set(uint8_t r, uint8_t g, uint8_t b) {
    if (!_strip) return;
    _r = r; _g = g; _b = b;
    _strip->setPixelColor(0, _strip->Color(g, r, b));
    _strip->show();
  }

  void off() {
    set(0, 0, 0);
  }

  uint8_t r() const { return _r; }
  uint8_t g() const { return _g; }
  uint8_t b() const { return _b; }

private:
  int _pin = -1;
  uint16_t _count = 1;
  uint8_t _r = 0, _g = 0, _b = 0;
  Adafruit_NeoPixel* _strip = nullptr;
};