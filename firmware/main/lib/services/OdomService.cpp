#include "OdomService.h"
#include <math.h>

bool OdomService::start(PoseBus& pose, int hz) {
  pose_ = &pose;
  hz_ = hz;
  return CreateTaskPinnedPSRAM("odom", &OdomService::taskThunk, 6, 3, &th_, 1, this);
}

void OdomService::correctYawRad(float target_yaw_rad) {
  if (!imuReady_ || !pose_) return;
  float delta_deg = (target_yaw_rad - pose_->get().yaw_rad) * (180.0f / (float)M_PI);
  while (delta_deg >  180.0f) delta_deg -= 360.0f;
  while (delta_deg < -180.0f) delta_deg += 360.0f;
  imu_.adjustYawOffsetDeg(delta_deg);
}

bool OdomService::prepareForSleep() {
  if (!imuReady_) return true;
  return imu_.sleep();
}

void OdomService::run() {
  Pose2D p;
  vTaskDelay(pdMS_TO_TICKS(500));

  // Cooperative init/retry loop so sleep requests can still quiesce the task
  // even if BMI270 init is failing.
  while (!imuReady_) {
    if (sleepRequested_) {
      idle_ = true;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    idle_ = false;

    if (imu_.begin()) {
      imu_.setInvertYaw(true);   // IMU Z-axis is physically inverted on this board
      // Preserve the EKF yaw across reinit so a transient I2C failure doesn't
      // reset the robot's heading back to 0° and disrupt EKF convergence.
      const float ekfYawDeg = pose_ ? (pose_->get().yaw_rad * (180.0f / (float)M_PI)) : 0.0f;
      imu_.setYawOffsetDeg(ekfYawDeg);
      imuReady_ = true;
      Serial.println("[Odom] BMI270 init OK");
      break;
    }

    Serial.println("[Odom] BMI270 init FAILED");
    // Back off and retry later instead of getting stuck forever.
    for (int i = 0; i < 20; ++i) {
      if (sleepRequested_) {
        idle_ = true;
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }

  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS((hz_ > 0) ? (1000u / (uint32_t)hz_) : 4u);

  for (;;) {
    if (sleepRequested_) {
      idle_ = true;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    idle_ = false;

    if (!imuReady_) {
      // Retry init cooperatively in case the IMU was unavailable at boot/wake.
      if (imu_.begin()) {
        imu_.setInvertYaw(true);   // IMU Z-axis is physically inverted on this board
        imu_.setYawOffsetDeg(0.0f);
        imuReady_ = true;
        Serial.println("[Odom] BMI270 init OK");
      } else {
        vTaskDelay(pdMS_TO_TICKS(100));
        continue;
      }
    }

    float yawDeg = 0.0f;
    if (imu_.readYawDeg(yawDeg)) {
      // Read-modify-write: preserve x, y owned by PoseService
      Pose2D p = pose_->get();
      p.yaw_rad = yawDeg * (float)M_PI / 180.0f;
      pose_->update(p);
    }

    vTaskDelayUntil(&last, period);
  }
}
