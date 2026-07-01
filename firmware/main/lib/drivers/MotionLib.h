#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <limits.h>
#include <string.h>
#include <math.h> // lroundf

/**
 * MotionLib — 4-motor mecanum/omni driver on PCA9685.
 *
 * NEW DRIVER MODEL:
 *   Each motor uses:
 *     - 1 PWM channel for speed
 *     - 2 logic channels for direction
 *
 * Example TB6612-style mapping:
 *   PWM, IN1, IN2
 *
 * Notes:
 * - Uses global Wire. This library NEVER calls Wire.begin() or setClock().
 * - Initialize I²C in your main sketch before calling begin().
 */
class MotionLib {
public:
  struct Config {
    uint8_t  i2cAddr         = 0x40;
    uint16_t pwmFreqHz       = 1400;

    // Direction-logic polarity. Usually false for TB6612-style inputs.
    bool     logicActiveLow  = false;

    // Cap for motor speed commands
    float    maxDuty01       = 0.5f;


    // Physical motor order used throughout this firmware:
    //   0 = Back Right
    //   1 = Front Right
    //   2 = Back Left
    //   3 = Front Left
    uint8_t pwm[4] = {7, 1, 8, 15};
    uint8_t in1[4] = {5, 4, 12, 13};
    uint8_t in2[4] = {6, 2, 9, 14};

    // Per-motor polarity fix:
    // +1 = normal, -1 = reverse that motor logically
    int8_t  motorPolarity[4] = {+1, +1, +1, +1};

    int     weakIdxDefault   = -1;
  };

  MotionLib() : _cfg(), _pwm(_cfg.i2cAddr) {}
  explicit MotionLib(const Config& cfg) : _cfg(cfg), _pwm(_cfg.i2cAddr) {}

  bool begin();

  // ---- Low-level control (motor index 0..3) ----
  void motorSet(uint8_t motor, int8_t dir, float speed01);
  void motorBrake(uint8_t motor);
  void motorsAll(int8_t dir, float speed01);
  void motorsStopAll();

  // ---- Motion helpers ----
  static inline float clamp01(float s){ return s < 0 ? 0 : (s > 1 ? 1 : s); }

  void turnLeft (float speed01);
  void turnRight(float speed01);

  void forward (float speed01);
  void backward(float speed01);

  void strafeLeft (float speed01);
  void strafeRight(float speed01);

  // ---- Staggered start helpers ----
  void forward_staggered (uint16_t ms, int weakIdx = INT_MIN);
  void backward_staggered(uint16_t ms, int weakIdx = INT_MIN);
  void turnLeft_staggered (uint16_t ms, int weakIdx = INT_MIN);
  void turnRight_staggered(uint16_t ms, int weakIdx = INT_MIN);
  void strafeLeft_staggered (uint16_t ms, int weakIdx = INT_MIN);
  void strafeRight_staggered(uint16_t ms, int weakIdx = INT_MIN);

  // Omni mix: longitudinal (forward +), lateral (right +), rot (CCW/left +), cap
  void drive(float vertical, float horizontal, float rot, float maxSpeed01);

  // ---- Demo sequence (optional) ----
  void runTestSequence(uint16_t preMs = 5);

  // ---- Config mutators ----
  void setLogicActiveLow(bool v)     { _cfg.logicActiveLow = v; }
  void setMaxDuty01(float v)         { _cfg.maxDuty01 = constrain(v, 0.0f, 1.0f); }
  void setPWMFreq(uint16_t hz)       { _cfg.pwmFreqHz = hz; if (_begun) _pwm.setPWMFreq(_cfg.pwmFreqHz); }
  void setWeakIdxDefault(int idx)    { _cfg.weakIdxDefault = (idx >= 0 && idx < 4) ? idx : -1; }

  void setChannelMap(const uint8_t pwm[4], const uint8_t in1[4], const uint8_t in2[4]) {
    memcpy(_cfg.pwm, pwm, sizeof(_cfg.pwm));
    memcpy(_cfg.in1, in1, sizeof(_cfg.in1));
    memcpy(_cfg.in2, in2, sizeof(_cfg.in2));
  }

  void setMotorPolarity(const int8_t pol[4]) {
    memcpy(_cfg.motorPolarity, pol, sizeof(_cfg.motorPolarity));
  }

  const Config& config() const { return _cfg; }
        Config& config()       { return _cfg; }

private:
  inline uint16_t dutyFrom01(float s) const {
    s = constrain(s, 0.0f, 1.0f);
    return (uint16_t)lroundf(s * 4095.0f);
  }

  inline void setPWM01(uint8_t ch, float s) {
    _pwm.setPin(ch, dutyFrom01(s));
  }

  inline void setLogic(uint8_t ch, bool high) {
    bool out = _cfg.logicActiveLow ? !high : high;
    _pwm.setPin(ch, out ? 4095 : 0);
  }

  inline int resolveWeak(int weakIdx) const {
    return (weakIdx == INT_MIN) ? _cfg.weakIdxDefault : weakIdx;
  }

private:
  Config _cfg;
  Adafruit_PWMServoDriver _pwm;
  bool _begun = false;
};