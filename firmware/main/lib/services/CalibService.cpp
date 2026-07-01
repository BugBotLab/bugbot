#include "CalibService.h"
#include "MotionService.h"
#include <LittleFS.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

bool CalibService::begin(MotionService& motion, PoseBus& bus, WebWS& ws) {
  motion_ = &motion;
  bus_    = &bus;
  ws_     = &ws;
  return true;
}

bool CalibService::loadFromFlash() {
  File f = LittleFS.open("/config/kinematics.cfg", "r");
  if (!f) return false;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    const int eq = line.indexOf('=');
    if (eq < 0) continue;
    const String key = line.substring(0, eq);
    const float  val = line.substring(eq + 1).toFloat();
    if (key == "k_lin_fwd")  model_.k_lin_fwd_mm_s = val;
    if (key == "k_lin_lat")  model_.k_lin_lat_mm_s = val;
    if (key == "k_rot")      model_.k_rot_rad_s    = val;
    if (key == "calibrated") model_.calibrated      = (val != 0.0f);
  }
  f.close();
  if (model_.calibrated) {
    Serial.printf("[Calib] loaded: fwd=%.1f lat=%.1f rot=%.4f mm/s & rad/s\n",
                  model_.k_lin_fwd_mm_s, model_.k_lin_lat_mm_s, model_.k_rot_rad_s);
  }
  return model_.calibrated;
}

bool CalibService::saveToFlash_() {
  File f = LittleFS.open("/config/kinematics.cfg", "w");
  if (!f) { Serial.println("[Calib] ERROR: cannot open kinematics.cfg for write"); return false; }
  f.printf("k_lin_fwd=%.3f\n",  model_.k_lin_fwd_mm_s);
  f.printf("k_lin_lat=%.3f\n",  model_.k_lin_lat_mm_s);
  f.printf("k_rot=%.6f\n",      model_.k_rot_rad_s);
  f.printf("calibrated=1\n");
  f.close();
  return true;
}

void CalibService::trigger() {
  if (isRunning()) return;
  phase_      = WAIT_TAG;
  phaseStart_ = millis();
  tagAccum_.reset();
  Serial.println("[Calib] started — place robot ~300mm in front of AprilTag");
  sendStatus_(0, "waiting_tag");
}

void CalibService::cancel() {
  if (!isRunning()) return;
  motion_->setCommandVec(0, 0, 0);
  phase_ = IDLE;
  char buf[64];
  snprintf(buf, sizeof(buf), "{\"calib_status\":\"idle\"}");
  ws_->sendText(buf);
  Serial.println("[Calib] cancelled");
}

bool CalibService::isRunning() const {
  return phase_ != IDLE && phase_ != DONE && phase_ != CALIB_ERROR;
}

void CalibService::onAprilTags(uint8_t n, const WebWS::AprilTagHit* hits) {
  if (n == 0 || !isRunning()) return;
  const auto& h = hits[0];
  if (h.dist_mm < 50.0f) return;
  portENTER_CRITICAL(&tagMux_);
  pendingTag_ = { h.id, h.az_deg, h.dist_mm, true };
  portEXIT_CRITICAL(&tagMux_);
}

void CalibService::advance_(Phase next, uint32_t now) {
  phase_      = next;
  phaseStart_ = now;
  tagAccum_.reset();
}

void CalibService::error_(const char* msg) {
  phase_ = CALIB_ERROR;
  motion_->setCommandVec(0, 0, 0);
  sendStatus_(0, nullptr, msg);
  Serial.printf("[Calib] ERROR: %s\n", msg);
}

void CalibService::sendStatus_(int progress, const char* phase, const char* err) {
  if (!ws_) return;
  char buf[200];
  if (err) {
    snprintf(buf, sizeof(buf),
             "{\"calib_status\":\"error\",\"msg\":\"%s\"}", err);
  } else if (phase_ == DONE) {
    snprintf(buf, sizeof(buf),
             "{\"calib_status\":\"done\","
             "\"k_lin_fwd\":%.1f,\"k_lin_lat\":%.1f,\"k_rot\":%.4f}",
             model_.k_lin_fwd_mm_s, model_.k_lin_lat_mm_s, model_.k_rot_rad_s);
  } else if (!phase || phase_ == IDLE) {
    snprintf(buf, sizeof(buf), "{\"calib_status\":\"idle\"}");
  } else {
    snprintf(buf, sizeof(buf),
             "{\"calib_status\":\"running\",\"phase\":\"%s\",\"progress\":%d}",
             phase, progress);
  }
  ws_->sendText(buf);
}

bool CalibService::bestTag_(float& dist, float& az, uint32_t now) const {
  if (tagAccum_.ok()) {
    dist = tagAccum_.dist();
    az   = tagAccum_.az();
    return true;
  }
  if (lastTag_.valid && (now - lastTag_.t_ms) < 3000) {
    dist = lastTag_.dist_mm;
    az   = lastTag_.az_deg;
    return true;
  }
  return false;
}

// Pre-rotate a body-frame (fwd, lat) command by the inverse of the current EKF
// yaw so that MotionService's field-centric R(-yaw) rotation cancels out, leaving
// the robot with the intended body-frame motion.  Pure rotation commands are
// field-centric-agnostic and are passed straight through as the rot argument.
void CalibService::setBodyCmd_(float fwd, float lat, float rot) {
  const float yaw = bus_->get().yaw_rad;
  const float cy  = cosf(yaw), sy = sinf(yaw);
  motion_->setCommandVec(cy*fwd - sy*lat, sy*fwd + cy*lat, rot);
}

void CalibService::tick() {
  const uint32_t now = millis();

  // Consume latest pending tag sample
  PendingTag tag{};
  portENTER_CRITICAL(&tagMux_);
  tag               = pendingTag_;
  pendingTag_.fresh = false;
  portEXIT_CRITICAL(&tagMux_);

  // Update rolling cache and accumulate during settle/sample phases
  if (tag.fresh && tag.dist_mm > 50.0f) {
    lastTag_ = { tag.az_deg, tag.dist_mm, now, true };
    switch (phase_) {
      case WAIT_TAG:
      case FWD_PRE:  case FWD_POST:
      case ROT_PRE:
      case LAT_PRE:  case LAT_POST:
        tagAccum_.dist_sum += tag.dist_mm;
        tagAccum_.az_sum   += tag.az_deg;
        tagAccum_.n++;
        break;
      default: break;
    }
  }

  switch (phase_) {
    case IDLE: case DONE: case CALIB_ERROR: return;

    // ── Wait for initial tag visibility ────────────────────────────────────
    case WAIT_TAG:
      if (tag.fresh) {
        Serial.println("[Calib] tag acquired — running forward phase");
        sendStatus_(5, "forward");
        advance_(FWD_PRE, now);
      } else if (now - phaseStart_ > kWaitMs) {
        error_("No AprilTag visible. Place robot ~300 mm in front of a tag.");
      }
      return;

    // ── Forward calibration ────────────────────────────────────────────────
    case FWD_PRE:
      if (now - phaseStart_ > kSettleMs) {
        float d, a;
        if (!bestTag_(d, a, now)) { error_("Tag not visible — point camera at AprilTag"); return; }
        distBefore_     = d;
        moveTargetYaw_  = bus_->get().yaw_rad;  // lock heading for the move
        Serial.printf("[Calib] fwd pre: dist=%.1f mm  yaw=%.3f (n=%d)\n",
                      distBefore_, moveTargetYaw_, tagAccum_.n);
        sendStatus_(10, "forward");
        setBodyCmd_(kPower, 0, 0);
        advance_(FWD_MOVE, now);
      }
      return;

    case FWD_MOVE:
      if (now - phaseStart_ > kMoveMs) {
        motion_->setCommandVec(0, 0, 0);
        sendStatus_(20, "forward");
        advance_(FWD_POST, now);
      } else {
        // IMU heading correction: keep the robot going straight
        const float yawErr  = wrapPi_(bus_->get().yaw_rad - moveTargetYaw_);
        const float rotCorr = constrain(-yawErr * kHeadingKp, -0.4f, 0.4f);
        setBodyCmd_(kPower, 0, rotCorr);
      }
      return;

    case FWD_POST:
      if (now - phaseStart_ > kSettleMs) {
        float d, a;
        if (!bestTag_(d, a, now)) { error_("Tag lost after forward move — ensure tag stays in view"); return; }
        distAfter_ = d;
        Serial.printf("[Calib] fwd post: dist=%.1f mm  Δ=%.1f mm (n=%d)\n",
                      distAfter_, distBefore_ - distAfter_, tagAccum_.n);
        sendStatus_(30, "returning");
        setBodyCmd_(-kPower, 0, 0);
        advance_(FWD_BACK, now);
      }
      return;

    case FWD_BACK:
      if (now - phaseStart_ > kMoveMs) {
        motion_->setCommandVec(0, 0, 0);
        advance_(FWD_BACK_SETTLE, now);
      } else {
        const float yawErr  = wrapPi_(bus_->get().yaw_rad - moveTargetYaw_);
        const float rotCorr = constrain(-yawErr * kHeadingKp, -0.4f, 0.4f);
        setBodyCmd_(-kPower, 0, rotCorr);
      }
      return;

    case FWD_BACK_SETTLE:
      if (now - phaseStart_ > kSettleMs) {
        sendStatus_(40, "rotation");
        advance_(ROT_PRE, now);
      }
      return;

    // ── Rotation calibration ───────────────────────────────────────────────
    case ROT_PRE:
      if (now - phaseStart_ > kSettleMs) {
        yawBefore_ = bus_->get().yaw_rad;
        Serial.printf("[Calib] rot pre: yaw=%.4f rad\n", yawBefore_);
        sendStatus_(45, "rotation");
        motion_->setCommandVec(0, 0, kPower);
        advance_(ROT_CW, now);
      }
      return;

    case ROT_CW: {
      // Angle-based termination: stop when 45° reached or safety timeout
      const float dYaw = wrapPi_(bus_->get().yaw_rad - yawBefore_);
      if (fabsf(dYaw) >= kTargetRotRad || now - phaseStart_ > kRotTimeoutMs) {
        rotMoveMs_ = now - phaseStart_;
        motion_->setCommandVec(0, 0, 0);
        Serial.printf("[Calib] rot CW done: Δyaw=%.4f rad in %lu ms\n", dYaw, (unsigned long)rotMoveMs_);
        advance_(ROT_POST, now);
      }
      return;
    }

    case ROT_POST:
      if (now - phaseStart_ > kSettleMs) {
        yawAfter_ = bus_->get().yaw_rad;
        Serial.printf("[Calib] rot post (settled): yaw=%.4f rad\n", yawAfter_);
        sendStatus_(55, "rotation_return");
        motion_->setCommandVec(0, 0, -kPower);
        advance_(ROT_BACK, now);
      }
      return;

    case ROT_BACK: {
      // Return to original heading using IMU
      const float dYaw = wrapPi_(bus_->get().yaw_rad - yawBefore_);
      if (fabsf(dYaw) <= 0.05f || now - phaseStart_ > kRotTimeoutMs) {
        motion_->setCommandVec(0, 0, 0);
        advance_(ROT_BACK_SETTLE, now);
      }
      return;
    }

    case ROT_BACK_SETTLE:
      if (now - phaseStart_ > kSettleMs) {
        sendStatus_(65, "lateral");
        advance_(LAT_PRE, now);
      }
      return;

    // ── Lateral calibration ────────────────────────────────────────────────
    case LAT_PRE:
      if (now - phaseStart_ > kSettleMs) {
        float d, a;
        if (!bestTag_(d, a, now)) { error_("Tag not visible before lateral move"); return; }
        latAzBefore_   = a;
        latDistAvg_    = d;
        moveTargetYaw_ = bus_->get().yaw_rad;
        Serial.printf("[Calib] lat pre: az=%.2f°  dist=%.1f mm  yaw=%.3f (n=%d)\n",
                      latAzBefore_, latDistAvg_, moveTargetYaw_, tagAccum_.n);
        sendStatus_(70, "lateral");
        setBodyCmd_(0, kPower, 0);
        advance_(LAT_MOVE, now);
      }
      return;

    case LAT_MOVE:
      if (now - phaseStart_ > kLatMoveMs) {
        motion_->setCommandVec(0, 0, 0);
        advance_(LAT_POST, now);
      } else {
        const float yawErr  = wrapPi_(bus_->get().yaw_rad - moveTargetYaw_);
        const float rotCorr = constrain(-yawErr * kHeadingKp, -0.4f, 0.4f);
        setBodyCmd_(0, kPower, rotCorr);
      }
      return;

    case LAT_POST:
      if (now - phaseStart_ > kSettleMs) {
        float d, a;
        if (!bestTag_(d, a, now)) { error_("Tag lost after lateral move"); return; }
        latAzAfter_ = a;
        latDistAvg_ = (latDistAvg_ + d) * 0.5f;
        Serial.printf("[Calib] lat post: az=%.2f°  Δaz=%.2f° (n=%d)\n",
                      latAzAfter_, latAzBefore_ - latAzAfter_, tagAccum_.n);
        sendStatus_(80, "lateral_return");
        setBodyCmd_(0, -kPower, 0);
        advance_(LAT_BACK, now);
      }
      return;

    case LAT_BACK:
      if (now - phaseStart_ > kLatMoveMs) {
        motion_->setCommandVec(0, 0, 0);
        advance_(LAT_BACK_SETTLE, now);
      } else {
        const float yawErr  = wrapPi_(bus_->get().yaw_rad - moveTargetYaw_);
        const float rotCorr = constrain(-yawErr * kHeadingKp, -0.4f, 0.4f);
        setBodyCmd_(0, -kPower, rotCorr);
      }
      return;

    case LAT_BACK_SETTLE:
      if (now - phaseStart_ > kSettleMs) {
        advance_(FIT_SAVE, now);
      }
      return;

    // ── Compute model and save ─────────────────────────────────────────────
    case FIT_SAVE: {
      const float T_move = kMoveMs    * 0.001f;
      const float T_lat  = kLatMoveMs * 0.001f;

      // Forward: camera faces forward, robot moves TOWARD the tag → distAfter_ < distBefore_.
      // delta_dist > 0, k_lin_fwd_mm_s is naturally positive.
      const float delta_dist = distBefore_ - distAfter_;
      if (fabsf(delta_dist) < 2.0f) {
        error_("Forward movement too small — check motors and tag distance");
        return;
      }
      model_.k_lin_fwd_mm_s = delta_dist / (kPower * T_move);

      // Rotation: use measured yaw change and actual time in ROT_CW phase
      float delta_yaw = yawAfter_ - yawBefore_;
      delta_yaw = wrapPi_(delta_yaw);
      const float T_rot = (rotMoveMs_ > 50) ? rotMoveMs_ * 0.001f : T_move;
      if (fabsf(delta_yaw) < 0.05f) {
        error_("Rotation too small — IMU may not be working or motors are stalled");
        return;
      }
      model_.k_rot_rad_s = delta_yaw / (kPower * T_rot);

      // Lateral: az shift at known distance → lateral displacement
      const float delta_az_rad = (latAzBefore_ - latAzAfter_) * ((float)M_PI / 180.0f);
      const float lat_dist_mm  = latDistAvg_ * sinf(fabsf(delta_az_rad));
      if (fabsf(delta_az_rad) > 0.05f && lat_dist_mm > 5.0f) {
        // Camera faces forward: robot strafes right → tag shifts left in camera → delta_az > 0,
        // lat_dist_mm > 0, k_lin_lat_mm_s is naturally positive.
        model_.k_lin_lat_mm_s = lat_dist_mm / (kPower * T_lat);
      } else {
        // Omni wheels: lateral is typically similar to forward
        model_.k_lin_lat_mm_s = model_.k_lin_fwd_mm_s;
        Serial.println("[Calib] lateral signal weak — using fwd scale for lat");
      }

      model_.calibrated = true;
      saveToFlash_();

      Serial.printf("[Calib] DONE  k_fwd=%.1f  k_lat=%.1f  k_rot=%.4f  (mm/s & rad/s per unit)\n",
                    model_.k_lin_fwd_mm_s, model_.k_lin_lat_mm_s, model_.k_rot_rad_s);
      phase_ = DONE;
      sendStatus_(100);
      return;
    }
  }
}
