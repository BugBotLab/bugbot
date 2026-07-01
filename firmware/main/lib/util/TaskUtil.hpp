// TaskUtil.hpp
// FreeRTOS task creation helpers: CreateTaskPinned (DRAM stack) and
// CreateTaskPinnedPSRAM (PSRAM stack, frees DRAM). Periodic() drives fixed-rate loops.
#pragma once
#include <Arduino.h>
#include <freertos/idf_additions.h>

constexpr size_t STACK_KB(size_t kb){ return (kb * 1024) / sizeof(StackType_t); }

// Create a pinned task with DRAM stack (legacy — prefer CreateTaskPinnedPSRAM).
inline bool CreateTaskPinned(const char* name,
                             TaskFunction_t fn,
                             uint32_t stack_kb,
                             UBaseType_t prio,
                             TaskHandle_t* outHandle,
                             BaseType_t core,
                             void* arg = nullptr)
{
  return xTaskCreatePinnedToCore(fn, name, STACK_KB(stack_kb),
                                 arg, prio, outHandle, core) == pdPASS;
}

// Create a pinned task with stack allocated from PSRAM — frees ~stack_kb of DRAM.
inline bool CreateTaskPinnedPSRAM(const char* name,
                                   TaskFunction_t fn,
                                   uint32_t stack_kb,
                                   UBaseType_t prio,
                                   TaskHandle_t* outHandle,
                                   BaseType_t core,
                                   void* arg = nullptr)
{
  return xTaskCreatePinnedToCoreWithCaps(fn, name, STACK_KB(stack_kb),
                                          arg, prio, outHandle, core,
                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) == pdPASS;
}

template<typename F>
inline void Periodic(uint32_t period_ms, F&& body)
{
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(period_ms);
  for (;;) { body(); vTaskDelayUntil(&last, period); }
}
