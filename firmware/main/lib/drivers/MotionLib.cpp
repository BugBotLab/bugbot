#include "MotionLib.h"

// -------------------- Init --------------------
bool MotionLib::begin() {
  // Do NOT call Wire.begin()/setClock() here.
  _pwm.begin();
  _pwm.setPWMFreq(_cfg.pwmFreqHz);

#ifndef MOTIONLIB_NO_SETOUTPUTMODE
#if __cplusplus >= 201103L
  _pwm.setOutputMode(true);
#endif
#endif

  motorsStopAll();
  _begun = true;
  return true;
}

// -------------------- Low-level --------------------
void MotionLib::motorSet(uint8_t motor, int8_t dir, float speed01) {
  if (motor > 3) return;

  speed01 = constrain(speed01, 0.0f, _cfg.maxDuty01);

  // Apply per-motor logical polarity correction
  dir = dir * _cfg.motorPolarity[motor];

  if (dir > 0) {
    // Forward
    setLogic(_cfg.in1[motor], true);
    setLogic(_cfg.in2[motor], false);
    setPWM01(_cfg.pwm[motor], speed01);
  }
  else if (dir < 0) {
    // Reverse
    setLogic(_cfg.in1[motor], false);
    setLogic(_cfg.in2[motor], true);
    setPWM01(_cfg.pwm[motor], speed01);
  }
  else {
    // Coast
    setLogic(_cfg.in1[motor], false);
    setLogic(_cfg.in2[motor], false);
    setPWM01(_cfg.pwm[motor], 0.0f);
  }
}

void MotionLib::motorBrake(uint8_t motor) {
  if (motor > 3) return;

  // Typical short-brake mode for TB6612-style driver
  setLogic(_cfg.in1[motor], true);
  setLogic(_cfg.in2[motor], true);
  setPWM01(_cfg.pwm[motor], 1.0f);
}

void MotionLib::motorsAll(int8_t dir, float speed01) {
  for (uint8_t m = 0; m < 4; ++m) motorSet(m, dir, speed01);
}

void MotionLib::motorsStopAll() {
  for (uint8_t m = 0; m < 4; ++m) motorSet(m, 0, 0.0f);
}

// -------------------- Motion helpers --------------------
// Physical motor order:
//   0 = Back Right
//   1 = Front Right
//   2 = Back Left
//   3 = Front Left
//
// Confirmed motion patterns from bench test:
//   forward      = [+,-,+,-]
//   backward     = [-,+,-,+]
//   strafe left  = [-,-,+,+]
//   strafe right = [+,+,-,-]
//   turn left    = [+,-,-,+]
//   turn right   = [-,+,+,-]
void MotionLib::turnLeft(float s) {
  s = clamp01(s);
  motorSet(0, +1, s);
  motorSet(1, -1, s);
  motorSet(2, -1, s);
  motorSet(3, +1, s);
}

void MotionLib::turnRight(float s) {
  s = clamp01(s);
  motorSet(0, -1, s);
  motorSet(1, +1, s);
  motorSet(2, +1, s);
  motorSet(3, -1, s);
}

void MotionLib::forward(float s) {
  s = clamp01(s);
  motorSet(0, +1, s);
  motorSet(1, -1, s);
  motorSet(2, +1, s);
  motorSet(3, -1, s);
}

void MotionLib::backward(float s) {
  s = clamp01(s);
  motorSet(0, -1, s);
  motorSet(1, +1, s);
  motorSet(2, -1, s);
  motorSet(3, +1, s);
}

void MotionLib::strafeLeft(float s) {
  s = clamp01(s);
  motorSet(0, -1, s);
  motorSet(1, -1, s);
  motorSet(2, +1, s);
  motorSet(3, +1, s);
}

void MotionLib::strafeRight(float s) {
  s = clamp01(s);
  motorSet(0, +1, s);
  motorSet(1, +1, s);
  motorSet(2, -1, s);
  motorSet(3, -1, s);
}

// -------------------- Staggered helpers --------------------
void MotionLib::forward_staggered(uint16_t ms, int weakIdx) {
  weakIdx = resolveWeak(weakIdx);
  if (weakIdx >= 0 && weakIdx < 4) {
    int8_t dir = (weakIdx == 0 || weakIdx == 1) ? +1 : -1;
    motorSet(weakIdx, dir, 1.0f);
    delay(ms);
  } else {
    motorSet(0, +1, 1.0f);
    motorSet(3, -1, 1.0f);
    delay(ms);
  }
  forward(1.0f);
}

void MotionLib::backward_staggered(uint16_t ms, int weakIdx) {
  weakIdx = resolveWeak(weakIdx);
  if (weakIdx >= 0 && weakIdx < 4) {
    int8_t dir = (weakIdx == 0 || weakIdx == 1) ? -1 : +1;
    motorSet(weakIdx, dir, 1.0f);
    delay(ms);
  } else {
    motorSet(0, -1, 1.0f);
    motorSet(3, +1, 1.0f);
    delay(ms);
  }
  backward(1.0f);
}

void MotionLib::turnLeft_staggered(uint16_t ms, int weakIdx) {
  weakIdx = resolveWeak(weakIdx);
  if (weakIdx >= 0 && weakIdx < 4) {
    motorSet(weakIdx, +1, 1.0f);
    delay(ms);
  } else {
    motorSet(0, +1, 1.0f);
    motorSet(2, +1, 1.0f);
    delay(ms);
  }
  turnLeft(1.0f);
}

void MotionLib::turnRight_staggered(uint16_t ms, int weakIdx) {
  weakIdx = resolveWeak(weakIdx);
  if (weakIdx >= 0 && weakIdx < 4) {
    motorSet(weakIdx, -1, 1.0f);
    delay(ms);
  } else {
    motorSet(1, -1, 1.0f);
    motorSet(3, -1, 1.0f);
    delay(ms);
  }
  turnRight(1.0f);
}

void MotionLib::strafeLeft_staggered(uint16_t ms, int weakIdx) {
  weakIdx = resolveWeak(weakIdx);
  if (weakIdx >= 0 && weakIdx < 4) {
    int8_t dir = (weakIdx == 0 || weakIdx == 2) ? +1 : -1;
    motorSet(weakIdx, dir, 1.0f);
    delay(ms);
  } else {
    motorSet(0, +1, 1.0f);
    motorSet(2, +1, 1.0f);
    delay(ms);
  }
  strafeLeft(1.0f);
}

void MotionLib::strafeRight_staggered(uint16_t ms, int weakIdx) {
  weakIdx = resolveWeak(weakIdx);
  if (weakIdx >= 0 && weakIdx < 4) {
    int8_t dir = (weakIdx == 1 || weakIdx == 3) ? +1 : -1;
    motorSet(weakIdx, dir, 1.0f);
    delay(ms);
  } else {
    motorSet(1, +1, 1.0f);
    motorSet(3, +1, 1.0f);
    delay(ms);
  }
  strafeRight(1.0f);
}

// -------------------- Omni drive mix --------------------
void MotionLib::drive(float longitudinal, float lateral, float rot, float maxSpeed01) {
  longitudinal = constrain(longitudinal, -1.0f, 1.0f);
  lateral      = constrain(lateral,      -1.0f, 1.0f);
  rot          = constrain(rot,          -1.0f, 1.0f);

  // Basis vectors in physical motor order (BR, FR, BL, FL):
  //   forward      = [+,-,+,-]
  //   strafe right = [+,+,-,-]
  //   turn left    = [+,-,-,+]
  float d0 = +longitudinal + lateral + rot; // BR
  float d1 = -longitudinal + lateral - rot; // FR
  float d2 = +longitudinal - lateral - rot; // BL
  float d3 = -longitudinal - lateral + rot; // FL

  float maxAbs = fmaxf(fmaxf(fabsf(d0), fabsf(d1)), fmaxf(fabsf(d2), fabsf(d3)));
  if (maxAbs < 1.0f) maxAbs = 1.0f;

  const float cap   = fminf(constrain(maxSpeed01, 0.0f, 1.0f), _cfg.maxDuty01);
  const float scale = cap / maxAbs;
  const float dead  = 0.02f;

  auto apply = [&](uint8_t m, float v) {
    if (fabsf(v) < dead) {
      motorSet(m, 0, 0.0f);
      return;
    }
    int8_t dir = (v >= 0.0f) ? +1 : -1;
    float sp = fabsf(v) * scale;
    motorSet(m, dir, sp);
  };

  apply(0, d0);
  apply(1, d1);
  apply(2, d2);
  apply(3, d3);
}

// -------------------- Demo sequence --------------------
void MotionLib::runTestSequence(uint16_t preMs) {
  turnLeft_staggered(preMs);
  delay(100);
  motorsStopAll();
  delay(1000);

  turnRight_staggered(preMs);
  delay(100);
  motorsStopAll();
  delay(1000);

  forward_staggered(preMs);
  delay(100);
  motorsStopAll();
  delay(1000);

  backward_staggered(preMs);
  delay(100);
  motorsStopAll();
  delay(1000);

  strafeLeft_staggered(preMs);
  delay(100);
  motorsStopAll();
  delay(1000);

  strafeRight_staggered(preMs);
  delay(100);
  motorsStopAll();
  delay(1000);
}