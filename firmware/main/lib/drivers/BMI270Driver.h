// BMI270Driver.h
// Gyroscope driver for the Bosch BMI270. Integrates Z-axis gyro rate into a yaw
// angle with bias compensation. Yaw offset can be corrected externally by PoseService.
#pragma once
#include <Arduino.h>
#include <Wire.h>

class BMI270Driver {
public:
  explicit BMI270Driver(uint8_t i2cAddr = 0x68, TwoWire& wire = Wire);

  bool begin();
  bool sleep();
  bool wake();
  bool readYawDeg(float& yawDegOut);
  bool recalibrateBias(int nSamples = 200);

  void setYawOffsetDeg(float offset);
  void adjustYawOffsetDeg(float delta_deg);
  void setInvertYaw(bool inv);
  void resetYaw();

  float getBiasZDps()      const { return _gyroBiasZ_dps; }
  float getFiltZDps()      const { return _gyroZFilt_dps; }
  float getAccelHorizMagG() const { return sqrtf(_accelXG*_accelXG + _accelYG*_accelYG); }

private:
  bool initBMI270_();
  bool readGyroDps_(float& gx, float& gy, float& gz);
  bool writeReg_(uint8_t reg, uint8_t value);
  bool readReg_(uint8_t reg, uint8_t& value);

private:
  uint8_t  _addr;
  TwoWire* _wire;

  float _yawDeg = 0.0f;
  float _yawOffsetDeg = 0.0f;
  bool  _invert = false;

  float _gyroBiasZ_dps = 0.0f;
  float _gyroZFilt_dps = 0.0f;
  float _prevGz        = 0.0f;
  bool  _prevGzValid   = false;

  float _accelXG = 0.0f;   // cached horizontal accel (g-units), updated each gyro read
  float _accelYG = 0.0f;

  uint32_t _lastUs = 0;
  bool _started = false;
};