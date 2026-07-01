#include "LidarLib.h"

#ifndef LL_DEBUG
#define LL_DEBUG 1
#endif
static void ll_log(const char* s){ if (LL_DEBUG) Serial.println(s); }

LidarLib::LidarLib() {}

bool LidarLib::probe_(uint8_t addr, TwoWire &wire) {
  wire.beginTransmission(addr);
  return (wire.endTransmission() == 0);
}

bool LidarLib::begin(TwoWire &wire) {
  // 1) Presence probe
  bool seen = false;
  for (int i = 0; i < 5 && !seen; ++i) { seen = probe_(0x29, wire); if (!seen) delay(10); }
  if (!seen) { ll_log("[LidarLib] probe 0x29 failed"); return false; }

  // 2) Driver begin — pass the wire instance so the SparkFun driver uses it
  bool ok = false;
  for (int tries = 0; tries < 3 && !ok; ++tries) { ok = _tof.begin(0x29, wire); if (!ok) delay(60); }
  if (!ok) { ll_log("[LidarLib] _tof.begin() failed"); return false; }

  // 3) Switch to 8×8 BEFORE startRanging.
  if (!_tof.setResolution(64)) {
    ll_log("[LidarLib] setResolution(64) FAILED — check sensor FW / I2C");
    return false;
  }
  delay(100);

  // Read back the resolution to confirm the change actually took effect.
  const uint8_t actualRes = _tof.getResolution();
  Serial.printf("[LidarLib] resolution after set: %d (%s)\r\n",
                (int)actualRes,
                (actualRes == 64) ? "8x8 OK" : "STILL 4x4 — setResolution ignored!");
  if (actualRes != 64) return false;

  if (!_tof.setRangingFrequency(_hz)) {
    Serial.printf("[LidarLib] setRangingFrequency(%d) rejected — sensor max may be lower\r\n", (int)_hz);
  }
  setRow(_row);

  Serial.printf("[LidarLib] begin ok — 8x8 mode, %d Hz\r\n", (int)_hz);
  return true;
}

void LidarLib::start() { if (!_started){ _tof.startRanging(); _started = true; } }
void LidarLib::stop()  { if (_started){ _tof.stopRanging();  _started = false; } }

void LidarLib::setRow(uint8_t row) { _row = (row < _imageH) ? row : (uint8_t)(_imageH - 1); }
uint8_t LidarLib::row() const { return _row; }

void LidarLib::setRangingFrequency(uint8_t hz) {
  _hz = hz;
  if (!_tof.setRangingFrequency(hz)) {
    Serial.printf("[LidarLib] setRangingFrequency(%d) rejected by sensor\r\n", (int)hz);
  }
}

bool LidarLib::dataReady() { return _tof.isDataReady(); }

bool LidarLib::getMeasurements(uint16_t out4[4]) {
  if (!_started) return false;
  if (!_tof.isDataReady()) return false;
  if (!_tof.getRangingData(&_results)) return false;
  extractRow4_(out4);
  return true;
}

bool LidarLib::getMeasurements(uint16_t out4[4], uint32_t timeout_ms) {
  if (!_started) return false;
  uint32_t t0 = millis();
  while (!_tof.isDataReady()) {
    if ((millis() - t0) >= timeout_ms) return false;
    delay(1);
  }
  if (!_tof.getRangingData(&_results)) return false;
  extractRow4_(out4);
  return true;
}

void LidarLib::extractRow4_(uint16_t out4[4]) const {
  const int row = _row;              // 0..3 (choose which horizontal row of the image to use)
  const int imageW = _imageW;        // 8 in 8×8 mode
  for (int i = 0; i < 4; i++) {
    int col = i * (imageW / 4);      // sample 4 evenly across 8 columns
    int idx = row * imageW + col;    // row-major index
    out4[i] = _results.distance_mm[idx];
  }
}

bool LidarLib::getGrid(uint16_t out[64]) {
  if (!_started) return false;
  if (!_tof.isDataReady()) return false;
  if (!_tof.getRangingData(&_results)) return false;
  for (int i = 0; i < 64; i++) {
    // target_status 5 or 9 = valid range (per ST datasheet); others = noise/no-target.
    const uint8_t st = _results.target_status[i];
    out[i] = (st == 5 || st == 9) ? (uint16_t)_results.distance_mm[i] : 0;
  }
  return true;
}

bool LidarLib::getGrid(uint16_t out[64], uint32_t timeout_ms) {
  if (!_started) return false;
  uint32_t t0 = millis();
  while (!_tof.isDataReady()) {
    if ((millis() - t0) >= timeout_ms) return false;
    delay(1);
  }
  if (!_tof.getRangingData(&_results)) return false;
  for (int i = 0; i < 64; i++) {
    const uint8_t st = _results.target_status[i];
    out[i] = (st == 5 || st == 9) ? (uint16_t)_results.distance_mm[i] : 0;
  }
  return true;
}

float LidarLib::elevationForRowRad(int r) const {
  // Row 0 = top (most upward, +elevation), row 7 = bottom (most downward, -elevation).
  // With fov_deg=60°: row 0=+26.25°, row 3=+3.75°, row 4=-3.75°, row 7=-26.25°.
  const float fov_rad = _cal.fov_deg * (float)M_PI / 180.0f;
  return (-0.5f * fov_rad) + ((7 - r + 0.5f) * (fov_rad / 8.0f));
}

float LidarLib::azimuthForColumnRad(int c) const {
  // c in 0..3, returns alpha in radians (sensor frame, +Y left)
  const float off_rad = _az.offset_deg * (float)M_PI / 180.0f;

  if (_az.use_lut) {
    float deg = (c >= 0 && c < 8) ? _az.lut_deg[c] : 0.0f;
    return deg * (float)M_PI / 180.0f + off_rad;
  }

  // Evenly spaced within effective FoV = fov_deg * scale, 8 columns
  const float fov_eff_rad = (_cal.fov_deg * _az.scale) * (float)M_PI / 180.0f;
  float alpha = (-0.5f * fov_eff_rad) + ((c + 0.5f) * (fov_eff_rad / 8.0f));
  return alpha + off_rad;
}
