#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>
#include <OpticalFlowESP32.h>

// ========== GroundSensors ==========
// Reads two optical flow sensors on a shared SPI bus and produces body-frame deltas.
// Body frame: forward = +Xb (from dy), left = +Yb (from dx).
class GroundSensors {
public:
  explicit GroundSensors(uint8_t spi_bus = 1);

  // Uses hard-coded XIAO ESP32-S3 defaults (defined in the .cpp)
  bool begin();

  // Configurable init for other boards/builds
  bool begin(int sck, int miso, int mosi,
             int cs,
             OpticalFlowESP32::Model model = OpticalFlowESP32::PMW3901,
             char orient = 'N',
             float baseline_mm = 34.0f,
             float mm_per_count = 0.03f,
             int8_t sign_dx = -1, int8_t sign_dy = -1,
             bool led_on = true);

  // Call once per tick
  void update();

  // Latest body-frame deltas for this tick (mm, mm, rad)
  void getBodyDelta(float& dxb_mm, float& dyb_mm) const;

  // Cumulative per-sensor totals (debug)
  void getPerSensorTotals(float& left_mm, float& forward_mm) const;

  // Basic config helpers
  inline void setScale(float mm_per_count)   { _k = mm_per_count; }
  inline void setBaseline(float baseline_mm) { _baseline = baseline_mm; }
  inline float scale() const                 { return _k; }
  inline float baseline() const              { return _baseline; }


private:
  void countsToBodyMM(int16_t dx, int16_t dy,int8_t sdx, int8_t sdy, float& fwd_mm, float& left_mm) const;

  // Shared SPI bus + two sensors
  SPIClass         _spi;
  OpticalFlowESP32 _flow;

  // Pins
  int _cs = -1;

  // Config (defaults; can be overridden at runtime)
  float  _baseline = 34.0f;   // mm (front↔back)
  float  _k        = 0.03f;   // mm/count
  int8_t _sign_dx = -1;
  int8_t _sign_dy = -1;

  // Last-tick body deltas
  float _dxb = 0.0f, _dyb = 0.0f;

  // Per-sensor totals (debug)
  float _left_mm = 0.0f, _fwd_mm = 0.0f;
};
