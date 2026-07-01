#include "GroundSensorsLib.h"
#include <Arduino.h>
#include <math.h>

// ================= Compile-time options =================
// Default strafe→yaw compensation (rad per mm). 0 disables correction.
// Tune experimentally (typical ~5e-4 .. 3e-3 rad/mm).


// ===== Hard-wired defaults for XIAO ESP32-S3 build =====
namespace {
  // Wiring (XIAO ESP32-S3)
  constexpr int PIN_SCK  = 7;   // D8
  constexpr int PIN_MISO = 8;   // D9
  constexpr int PIN_MOSI = 9;   // D10
  constexpr int PIN_CS = 2;     // CS

  // Optical model / orientation
  constexpr OpticalFlowESP32::Model MODEL = OpticalFlowESP32::PMW3901;
  constexpr char ORIENT = 'N';

  // Geometry & scaling
  constexpr float BASELINE_MM  = 34.0f;
  constexpr float MM_PER_COUNT = 0.03f;

  // Per-sensor signs (working mapping)
  constexpr int8_t SIGN_DX = +1;
  constexpr int8_t SIGN_DY = +1;

}

GroundSensors::GroundSensors(uint8_t spi_bus)
: _spi(spi_bus), _flow(_spi)
{
}

bool GroundSensors::begin() {
  // Apply hard-coded defaults
  _baseline = BASELINE_MM;
  _k        = MM_PER_COUNT;
  _sign_dx = SIGN_DX; 
  _sign_dy = SIGN_DY;

  return begin(PIN_SCK, PIN_MISO, PIN_MOSI,
               PIN_CS,
               MODEL, ORIENT,
               BASELINE_MM, MM_PER_COUNT,
               SIGN_DX, SIGN_DY,
               true /*led_on*/);
}

bool GroundSensors::begin(int sck, int miso, int mosi,
                          int cs,
                          OpticalFlowESP32::Model model,
                          char orient,
                          float baseline_mm,
                          float mm_per_count,
                          int8_t sign_dx,
                          int8_t sign_dy,
                          bool led_on)
{
  _cs = cs;
  _baseline   = baseline_mm;
  _k          = mm_per_count;
  _sign_dx   = (sign_dx >= 0) ? +1 : -1;
  _sign_dy   = (sign_dy >= 0) ? +1 : -1;

  _spi.begin(sck, miso, mosi);
  _spi.setHwCs(false);
  _spi.setFrequency(2'000'000);
  pinMode(_cs, OUTPUT); digitalWrite(_cs, HIGH);
  delay(2);

  const bool ok = _flow.begin(_spi, _cs, model, orient);

  if (led_on) { _flow.setLed(true); }

  return ok;
}

void GroundSensors::countsToBodyMM(int16_t dx, int16_t dy, int8_t sdx, int8_t sdy,
                                   float& fwd_mm, float& left_mm) const
{
  fwd_mm  = _k * float(sdy * dy);   // forward from signed dy
  left_mm = _k * float(sdx * dx);   // left    from signed dx
}

void GroundSensors::update() {
  // --- Read once per tick (PMW3901 clears on read) ---
  int16_t dx=0, dy=0;
  _flow.readMotionSimple(dx, dy);

  // --- Convert counts → body-frame mm (mapping/signs only; NO filtering) ---
  float f=0, l=0;
  countsToBodyMM(dx, dy, _sign_dx, _sign_dy, f, l);

  // Raw body deltas this tick (no bias, no deadband, no EMA, no clamps)
  const float dxb_raw  = f;
  const float dyb_raw  = l;
  


  // Publish raw linear + compensated yaw
  _dxb  = dxb_raw;
  _dyb  = dyb_raw;

  // --- Totals (debug) ---
  _left_mm += l; _fwd_mm += f;
}

void GroundSensors::getBodyDelta(float& dxb_mm, float& dyb_mm) const {
  dxb_mm = _dxb; dyb_mm = _dyb;
}

void GroundSensors::getPerSensorTotals(float& left_mm, float& forward_mm) const
{
  left_mm    = _left_mm;
  forward_mm = _fwd_mm;
}
