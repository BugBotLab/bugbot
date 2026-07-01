#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

/**
 * LidarLib — reads from VL53L5CX (8×8 mode, 64 zones).
 * Exposes full grid; callers select which rows/columns to use.
 */
class LidarLib {
public:
  struct Calibration {
    float fov_deg      = 45.0f;   // optical FoV (horizontal and vertical, assumed square)
    float x_off_mm     = 0.0f;    // sensor offset in robot frame
    float y_off_mm     = 0.0f;
    float z_off_mm     = 100.0f;  // sensor height above floor, mm
    float yaw_off_deg  = 180.0f;  // sensor→robot yaw
    bool  flip_lr      = true;    // physical horizontal flip
  };
  struct AzimuthModel {
    float scale        = 0.5f;    // FoV scale factor (0.92..1.02 typical)
    float offset_deg   = 0.0f;    // tiny per-row yaw trim
    bool  use_lut      = false;   // if true, use lut_deg[]
    float lut_deg[8]   = { -26.25f, -18.75f, -11.25f, -3.75f,
                            +3.75f, +11.25f, +18.75f, +26.25f };
  };

  LidarLib();

  // Presence probe + configure (defaults: row=1, hz=30)
  bool begin(TwoWire &wire = Wire);
  bool begin(uint8_t row, uint8_t rangingHz) { 
    if (!begin(Wire)) return false; 
    setRow(row); setRangingFrequency(rangingHz); 
    return true; 
  }

  void start();
  void stop();

  // Geometry / runtime config
  void setRow(uint8_t row);       // 0..7 in 8×8
  uint8_t row() const;
  void setRangingFrequency(uint8_t hz);
  uint8_t rangingHz() const { return _hz; }

  bool dataReady();

  // Measurements: 4 distances (mm) left→right in ROBOT frame (selected row only)
  bool getMeasurements(uint16_t out[4]);
  bool getMeasurements(uint16_t out[4], uint32_t ms);

  // Full 8×8 grid (row-major, row 0 = top): 64 distances, mm.
  // Returns false if data not ready or driver not started.
  bool getGrid(uint16_t out[64]);
  bool getGrid(uint16_t out[64], uint32_t ms);

  // Elevation angle (radians, positive = upward) for grid row r (0..7).
  // Assumes symmetric vertical FoV equal to horizontal FoV.
  float elevationForRowRad(int r) const;

  // ---- New: calibration + azimuth controls ----
  void setCalibration(const Calibration& c) { _cal = c; }
  Calibration calibration() const { return _cal; }

  void setFlipLR(bool on)              { _cal.flip_lr = on; }
  bool flipLR() const                  { return _cal.flip_lr; }

  void setFovDeg(float f)              { _cal.fov_deg = f; }
  float fovDeg() const                 { return _cal.fov_deg; }

  void setOffsetsMm(float x, float y)  { _cal.x_off_mm = x; _cal.y_off_mm = y; }
  float xOffsetMm() const              { return _cal.x_off_mm; }
  float yOffsetMm() const              { return _cal.y_off_mm; }

  void setYawOffsetDeg(float deg)      { _cal.yaw_off_deg = deg; }
  float yawOffsetDeg() const           { return _cal.yaw_off_deg; }

  void setAzimuthStraightening(float scale, float offset_deg) {
    _az.scale = scale; _az.offset_deg = offset_deg;
  }
  float azimScale() const              { return _az.scale; }
  float azimOffsetDeg() const          { return _az.offset_deg; }

  void useAzimuthLUT(bool use)         { _az.use_lut = use; }
  bool useAzimuthLUT() const           { return _az.use_lut; }

  void setAzimuthLUT(const float deg8[8]) {
    for (int i=0;i<8;i++) _az.lut_deg[i] = deg8[i];
  }
  float azimuthForColumnRad(int c) const;   // returns alpha for column c (0..7), radians

private:
  bool probe_(uint8_t addr, TwoWire &wire);
  void extractRow4_(uint16_t out4[4]) const;

  SparkFun_VL53L5CX _tof;
  mutable VL53L5CX_ResultsData _results{};
  uint8_t _row     = 1;     // default row
  uint8_t _hz      = 15;    // default Hz (8×8 mode max)
  bool    _started = false;

  // Fixed image geometry (8×8 mode)
  const int _imageW = 8;
  const int _imageH = 8;

  // New: config moved in here
  Calibration _cal{};
  AzimuthModel _az{};

};
