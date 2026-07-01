#include "MotionService.h"
#include "../core/BusLocks.hpp"
#include <math.h>

namespace {
static float clampSymmetric_(float v, float lim) {
  if (v < -lim) return -lim;
  if (v > lim) return lim;
  return v;
}

static float clamp01_(float v) {
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

static float clamp11_(float v) {
  return clampSymmetric_(v, 1.0f);
}

static float wrapDeg180_(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}
}


bool MotionService::wantsPeripheralPower(uint32_t nowMs) const {
  if (!autoPowerEnabled_) return true;
  const float cmdMag = fabsf(longitudinal_) + fabsf(lateral_) + fabsf(rotational_);
  if (cmdMag > 0.01f) return true;
  return (nowMs - lastNonZeroCommandMs_) < idleTimeoutMs_;
}

bool MotionService::start(PoseBus& pose, BoardPowerLib* power){
  pose_ = &pose;
  power_ = power;
  return CreateTaskPinnedPSRAM("motion", &MotionService::taskThunk, 10, 3, &th_, 1, this);
}

void MotionService::setCommand(DriveDir dir, float speed){
  speed = clamp11_(speed);
  switch (dir) {
    case DriveDir::Stop:    setCommandVec(0.0f, 0.0f, 0.0f); break;
    case DriveDir::Fwd:     setCommandVec(+speed, 0.0f, 0.0f); break;
    case DriveDir::Back:    setCommandVec(-speed, 0.0f, 0.0f); break;
    case DriveDir::StrafeL: setCommandVec(0.0f, -speed, 0.0f); break;
    case DriveDir::StrafeR: setCommandVec(0.0f, +speed, 0.0f); break;
    case DriveDir::TurnL:   setCommandVec(0.0f, 0.0f, +speed); break;
    case DriveDir::TurnR:   setCommandVec(0.0f, 0.0f, -speed); break;
  }
}

void MotionService::setCommandVec(float longitudinal, float lateral, float rotational){
  longitudinal_ = clampSymmetric_(longitudinal, cfg_.maxLinearSpeed > 0.0f ? cfg_.maxLinearSpeed : 1.0f);
  lateral_      = clampSymmetric_(lateral,      cfg_.maxLinearSpeed > 0.0f ? cfg_.maxLinearSpeed : 1.0f);
  rotational_   = clampSymmetric_(rotational,   cfg_.maxRotSpeed > 0.0f ? cfg_.maxRotSpeed : 1.0f);
  if (fabsf(longitudinal_) + fabsf(lateral_) + fabsf(rotational_) > 0.01f) {
    lastNonZeroCommandMs_ = millis();
    if (autoPowerEnabled_ && power_ && !power_->systemEnabled()) {
      power_->setSystemEnabled(true);
      needsReinit_ = true;  // PCA9685 needs re-init; settle delay happens in run()
    }
  }
  if (th_) xTaskNotifyGive(th_);
}

void MotionService::run(){
  MotionLib::Config cfg;
  cfg.pwmFreqHz = 950;
  cfg.maxDuty01 = 1.0f;
  cfg.logicActiveLow = false;

  cfg.pwm[0] = 7;
  cfg.pwm[1] = 1;
  cfg.pwm[2] = 8;
  cfg.pwm[3] = 15;

  cfg.in1[0] = 5;
  cfg.in1[1] = 4;
  cfg.in1[2] = 12;
  cfg.in1[3] = 13;

  cfg.in2[0] = 6;
  cfg.in2[1] = 2;
  cfg.in2[2] = 9;
  cfg.in2[3] = 14;

  cfg.motorPolarity[0] = +1;
  cfg.motorPolarity[1] = +1;
  cfg.motorPolarity[2] = +1;
  cfg.motorPolarity[3] = +1;

  MotionLib m(cfg);
  if (I2C_LOCK()) {
    m.begin();
    I2C_UNLOCK();
  }

  const float kCmdDead = clamp01_(cfg_.inputDeadband);
  const float kRotDead = clamp01_(cfg_.inputDeadband);
  const float kTranslateMinForHold = 0.18f;
  const float kYawDeadbandDeg = 1.8f;
  const float kYawKp   = cfg_.headingKp;
  const float kYawKd   = cfg_.headingKd;
  const float kYawCorrMax = 0.18f;
  const float kRespTargetDegPerSecPerCmd = 120.0f;
  const float kRespMinScale = 0.65f;
  const float kRespMaxScale = 1.20f;
  const float kRespAlpha = 0.10f;
  const float kCorrAlpha = 0.18f;
  const uint32_t waitTicks = pdMS_TO_TICKS((cfg_.controlRateHz > 0) ? (1000u / cfg_.controlRateHz) : 20u);

  float lastYawDeg = 0.0f;
  bool  lastYawValid = false;
  uint32_t lastTickMs = millis();
  uint32_t lastCommandMs = millis();
  float yawRateFiltDegPerSec = 0.0f;
  float responseFilt = kRespTargetDegPerSecPerCmd;
  float rotCorrFilt = 0.0f;

  float outLong = 0.0f, outLat = 0.0f, outRot = 0.0f;

  for(;;){
    ulTaskNotifyTake(pdTRUE, waitTicks);

    if (sleepRequested_) {
      if (I2C_LOCK(pdMS_TO_TICKS(100))) {
        m.motorsStopAll();
        I2C_UNLOCK();
      }
      idle_ = true;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    idle_ = false;

    if (needsReinit_) {
      needsReinit_ = false;
      vTaskDelay(pdMS_TO_TICKS(powerOnSettleMs_));  // wait for power rail to stabilise
      if (I2C_LOCK(pdMS_TO_TICKS(100))) {
        m.begin();
        I2C_UNLOCK();
      }
    }

    const uint32_t nowMs = millis();
    float longCmd = longitudinal_;
    float latCmd  = lateral_;
    float rotUser = rotational_;

    if (fabsf(longCmd) > 0.001f || fabsf(latCmd) > 0.001f || fabsf(rotUser) > 0.001f) {
      lastCommandMs = nowMs;
    } else if (cfg_.teleopTimeoutMs > 0 && (nowMs - lastCommandMs) > cfg_.teleopTimeoutMs) {
      longCmd = 0.0f;
      latCmd = 0.0f;
      rotUser = 0.0f;
    }

    const float transMag = sqrtf(longCmd * longCmd + latCmd * latCmd);
    const bool hasTranslation = transMag > kCmdDead;
    const bool headingHoldAllowed = transMag > kTranslateMinForHold;
    const bool hasRotationCmd = fabsf(rotUser) > kRotDead;

    float rotFinal = rotUser;
    float longRobot = longCmd;
    float latRobot  = latCmd;

    if (pose_) {
      const Pose2D p = pose_->get();
      const float yawRad = p.yaw_rad;
      const float yawDeg = yawRad * 180.0f / (float)M_PI;

      if (cfg_.fieldCentric) {
        const float cy = cosf(yawRad);
        const float sy = sinf(yawRad);
        longRobot =  longCmd * cy + latCmd * sy;
        latRobot  = -longCmd * sy + latCmd * cy;
      }

      float dt = (nowMs - lastTickMs) * 0.001f;
      if (dt < 0.001f) dt = 0.001f;
      if (dt > 0.10f) dt = 0.10f;
      lastTickMs = nowMs;

      float yawRateDegPerSec = 0.0f;
      if (lastYawValid) {
        yawRateDegPerSec = wrapDeg180_(yawDeg - lastYawDeg) / dt;
      }
      lastYawDeg = yawDeg;
      lastYawValid = true;
      yawRateFiltDegPerSec = 0.85f * yawRateFiltDegPerSec + 0.15f * yawRateDegPerSec;

      if (cfg_.headingHoldEnabled && headingHoldAllowed && !hasRotationCmd) {
        if (!headingHoldActive_) {
          headingTargetDeg_ = yawDeg;
          headingHoldActive_ = true;
          rotCorrFilt = 0.0f;
        }

        float yawErr = wrapDeg180_(headingTargetDeg_ - yawDeg);
        if (fabsf(yawErr) < kYawDeadbandDeg) yawErr = 0.0f;

        float yawCorrRaw = (kYawKp * yawErr) - (kYawKd * yawRateFiltDegPerSec);

        const float corrMag = fabsf(yawCorrRaw);
        if (corrMag > 0.03f) {
          float responseNow = fabsf(yawRateFiltDegPerSec) / (corrMag + 0.05f);
          responseFilt = (1.0f - kRespAlpha) * responseFilt + kRespAlpha * responseNow;
        }

        float respScale = kRespTargetDegPerSecPerCmd / (responseFilt + 1.0f);
        if (respScale < kRespMinScale) respScale = kRespMinScale;
        if (respScale > kRespMaxScale) respScale = kRespMaxScale;

        const float speedScale = 1.0f - 0.35f * fminf(transMag, 1.0f);

        float yawCorr = yawCorrRaw * respScale * speedScale;
        if (yawCorr >  kYawCorrMax) yawCorr =  kYawCorrMax;
        if (yawCorr < -kYawCorrMax) yawCorr = -kYawCorrMax;

        rotCorrFilt = (1.0f - kCorrAlpha) * rotCorrFilt + kCorrAlpha * yawCorr;
        rotFinal = rotCorrFilt;
      } else {
        headingHoldActive_ = false;
        headingTargetDeg_ = yawDeg;
        rotCorrFilt = 0.0f;
      }
    }

    if (cfg_.slewLimitEnabled) {
      const float dtSlew = (cfg_.controlRateHz > 0) ? (1.0f / (float)cfg_.controlRateHz) : 0.02f;
      const float maxStep = fmaxf(0.01f, cfg_.slewRate * dtSlew);
      const auto stepToward = [maxStep](float current, float target) {
        const float err = target - current;
        if (err > maxStep) return current + maxStep;
        if (err < -maxStep) return current - maxStep;
        return target;
      };
      outLong = stepToward(outLong, longRobot);
      outLat = stepToward(outLat, latRobot);
      outRot = stepToward(outRot, rotFinal);
    } else {
      outLong = longRobot;
      outLat = latRobot;
      outRot = rotFinal;
    }

    if (!I2C_LOCK(pdMS_TO_TICKS(50))) continue;

    if (power_ && !power_->systemEnabled()) {
      m.motorsStopAll();
      I2C_UNLOCK();
      continue;
    }

    if (!hasTranslation && !hasRotationCmd && fabsf(rotFinal) <= kCmdDead) {
      outLong = 0.0f;
      outLat = 0.0f;
      outRot = 0.0f;
      m.motorsStopAll();
    } else {
      m.drive(clamp11_(outLong), clamp11_(outLat), clamp11_(outRot), 1.0f);
    }

    I2C_UNLOCK();
  }
}
