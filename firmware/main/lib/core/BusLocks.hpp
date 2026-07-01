// BusLocks.hpp
// I2C_LOCK / I2C_UNLOCK helpers that serialize Wire transactions across FreeRTOS tasks.
// g_i2cMutex is defined in the main sketch and shared via extern.
#pragma once
#include <Arduino.h>

// Global I2C mutex is defined in Sketch.ino
extern SemaphoreHandle_t g_i2cMutex;

// Null-safe helpers to serialize I2C across tasks
inline bool I2C_LOCK(TickType_t to = portMAX_DELAY) {
  return (g_i2cMutex == nullptr) ? true : (xSemaphoreTake(g_i2cMutex, to) == pdTRUE);
}
inline void I2C_UNLOCK() {
  if (g_i2cMutex) xSemaphoreGive(g_i2cMutex);
}
