// I2CBus.h
// Wire bus wrapper with a FreeRTOS mutex. Use withLock() to safely share the I2C
// bus across tasks; all drivers that call Wire must acquire through this class.
#pragma once
#include <Arduino.h>
#include <Wire.h>
class I2CBus {
public:
  bool begin(uint32_t hz){ Wire.begin(); Wire.setClock(hz); return true; }
  void setTimeout(uint16_t ms){ Wire.setTimeOut(ms); }
  TwoWire& wire(){ return Wire; }
  template<typename F> auto withLock(F&& f) -> decltype(f()){
    xSemaphoreTake(mtx_, portMAX_DELAY);
    auto r = f();
    xSemaphoreGive(mtx_);
    return r;
  }
  I2CBus(){ mtx_ = xSemaphoreCreateMutex(); }
private:
  SemaphoreHandle_t mtx_{nullptr};
};
