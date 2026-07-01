#include "BMI270Driver.h"
#include <math.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include "../core/BusLocks.hpp"

// SparkFun library object
static BMI270 g_bmi270;

static constexpr uint8_t BMI270_REG_PWR_CONF = 0x7C;
static constexpr uint8_t BMI270_REG_PWR_CTRL = 0x7D;
static constexpr uint8_t BMI270_ADV_POWER_SAVE_EN = 0x02;
static constexpr uint8_t BMI270_PWR_CTRL_GYR_ACC_EN = 0x0E;

BMI270Driver::BMI270Driver(uint8_t i2cAddr, TwoWire& wire)
: _addr(i2cAddr), _wire(&wire) {}

bool BMI270Driver::begin() {
  if (!initBMI270_()) return false;

  const int N = 300;
  float sumZ = 0.0f;
  int good = 0;

  for (int i = 0; i < N; ++i) {
    float gx, gy, gz;
    if (readGyroDps_(gx, gy, gz) && fabsf(gz) < 2.0f) {
      sumZ += gz;
      ++good;
    }
    delay(4);
  }

  // Require at least 25% of samples to be near-zero — if fewer passed the motion
  // gate the robot was moving during calibration; keep the previous bias rather
  // than contaminating it with the rotation rate.
  if (good >= N / 4) {
    _gyroBiasZ_dps = sumZ / float(good);
  } else {
    Serial.println("[BMI270] WARN: motion during calibration, bias unchanged");
  }
  _gyroZFilt_dps = 0.0f;
  _yawDeg = 0.0f;
  _lastUs = micros();
  _started = true;

  Serial.printf("[BMI270] init OK addr=0x%02X biasZ=%.5f dps\n", _addr, _gyroBiasZ_dps);
  return true;
}

bool BMI270Driver::sleep() {
  if (!_started) return true;

  bool ok = true;
  ok &= writeReg_(BMI270_REG_PWR_CONF, 0x00);
  delay(1);
  ok &= writeReg_(BMI270_REG_PWR_CTRL, 0x00);
  delay(2);
  ok &= writeReg_(BMI270_REG_PWR_CONF, BMI270_ADV_POWER_SAVE_EN);
  delay(1);

  if (ok) {
    _started = false;
    Serial.println("[BMI270] sleep OK");
  } else {
    Serial.println("[BMI270] sleep FAILED");
  }
  return ok;
}

bool BMI270Driver::wake() {
  _started = false;
  return begin();
}

bool BMI270Driver::recalibrateBias(int nSamples) {
  if (!_started) return false;
  float sumZ = 0.0f;
  int good = 0;
  for (int i = 0; i < nSamples; ++i) {
    float gx, gy, gz;
    if (readGyroDps_(gx, gy, gz) && fabsf(gz) < 2.0f) { sumZ += gz; ++good; }
    delay(4);
  }
  if (good < nSamples / 4) return false;
  _gyroBiasZ_dps = sumZ / float(good);
  _gyroZFilt_dps = 0.0f;
  _prevGzValid   = false;
  Serial.printf("[BMI270] bias recal: %.5f dps (n=%d)\n", _gyroBiasZ_dps, good);
  return true;
}

bool BMI270Driver::readYawDeg(float& yawDegOut) {
  if (!_started) return false;

  float gx, gy, gz;
  if (!readGyroDps_(gx, gy, gz)) return false;

  // Reject sample if gz jumped by more than 50 dps since last read.
  // At 200 Hz a real rotation can't change that fast — larger jumps are I2C corruption.
  if (_prevGzValid && fabsf(gz - _prevGz) > 50.0f) {
    _prevGz = gz;
    return true;  // skip integration, return last known good yaw
  }
  _prevGz = gz;
  _prevGzValid = true;

  const uint32_t nowUs = micros();
  float dt = float(nowUs - _lastUs) * 1e-6f;
  _lastUs = nowUs;

  if (dt <= 0.0f || dt > 0.05f) {
    yawDegOut = _invert ? -_yawDeg + _yawOffsetDeg : _yawDeg + _yawOffsetDeg;
    while (yawDegOut < 0.0f)    yawDegOut += 360.0f;
    while (yawDegOut >= 360.0f) yawDegOut -= 360.0f;
    return true;
  }

  float gzCorr = gz - _gyroBiasZ_dps;

  const float alpha = 0.70f;
  _gyroZFilt_dps = alpha * _gyroZFilt_dps + (1.0f - alpha) * gzCorr;

  // Two-tier online bias estimator.
  // The static calibration at boot captures the cold bias, but the BMI270 bias
  // shifts ~0.1-0.3 dps as the chip reaches operating temperature.  When the
  // filtered rate is small the residual is predominantly the thermal bias drift,
  // not real rotation — use it to slowly re-anchor the bias estimate.
  if (fabsf(_gyroZFilt_dps) < 0.25f) {
    // Tier 1: clearly stationary — zero the output AND adapt bias quickly (30 s TC).
    _gyroBiasZ_dps += gzCorr * (1.0f / (200.0f * 30.0f));
    _gyroZFilt_dps = 0.0f;
  } else if (fabsf(_gyroZFilt_dps) < 0.60f) {
    // Tier 2: ambiguous — could be very slow rotation or residual bias.
    // Adapt bias at half speed (60 s TC) without zeroing the output.
    _gyroBiasZ_dps += gzCorr * (1.0f / (200.0f * 60.0f));
  }

  _yawDeg += _gyroZFilt_dps * dt;

  float yaw = _invert ? -_yawDeg : _yawDeg;
  yaw += _yawOffsetDeg;

  while (yaw < 0.0f)    yaw += 360.0f;
  while (yaw >= 360.0f) yaw -= 360.0f;

  yawDegOut = yaw;
  return true;
}

void BMI270Driver::setYawOffsetDeg(float offset) {
  _yawOffsetDeg = offset;
}

void BMI270Driver::adjustYawOffsetDeg(float delta_deg) {
  _yawOffsetDeg += delta_deg;
}

void BMI270Driver::setInvertYaw(bool inv) {
  _invert = inv;
}

void BMI270Driver::resetYaw() {
  _yawDeg = 0.0f;
  _gyroZFilt_dps = 0.0f;
  _lastUs = micros();
}

bool BMI270Driver::initBMI270_() {
  if (!I2C_LOCK(pdMS_TO_TICKS(5000))) {
    Serial.println("[BMI270] beginI2C lock timeout");
    return false;
  }

  int8_t rc = g_bmi270.beginI2C(_addr, *_wire);
  I2C_UNLOCK();

  if (rc != BMI2_OK) {
    Serial.printf("[BMI270] beginI2C failed rc=%d addr=0x%02X\n", rc, _addr);
    return false;
  }

  writeReg_(BMI270_REG_PWR_CTRL, BMI270_PWR_CTRL_GYR_ACC_EN);
  delay(2);
  return true;
}

bool BMI270Driver::readGyroDps_(float& gx, float& gy, float& gz) {
  if (!I2C_LOCK(pdMS_TO_TICKS(60))) return false;
  int8_t rc = g_bmi270.getSensorData();
  I2C_UNLOCK();
  if (rc != BMI2_OK) return false;

  gx = g_bmi270.data.gyroX;
  gy = g_bmi270.data.gyroY;
  gz = g_bmi270.data.gyroZ;
  _accelXG = g_bmi270.data.accelX;
  _accelYG = g_bmi270.data.accelY;

  // Reject NaN/Inf and physically impossible values (BMI270 max range is ±2000 dps).
  // Catches I2C bit errors that produce large-but-finite values which would
  // otherwise integrate into runaway heading drift.
  if (!isfinite(gx) || !isfinite(gy) || !isfinite(gz)) return false;
  if (fabsf(gx) > 2000.0f || fabsf(gy) > 2000.0f || fabsf(gz) > 2000.0f) return false;
  return true;
}

bool BMI270Driver::writeReg_(uint8_t reg, uint8_t value) {
  if (!I2C_LOCK(pdMS_TO_TICKS(100))) return false;
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  _wire->write(value);
  bool ok = (_wire->endTransmission() == 0);
  I2C_UNLOCK();
  return ok;
}

bool BMI270Driver::readReg_(uint8_t reg, uint8_t& value) {
  if (!I2C_LOCK(pdMS_TO_TICKS(100))) return false;
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) {
    I2C_UNLOCK();
    return false;
  }
  if (_wire->requestFrom((int)_addr, 1) != 1) {
    I2C_UNLOCK();
    return false;
  }
  value = _wire->read();
  I2C_UNLOCK();
  return true;
}
