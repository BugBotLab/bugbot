// PoseBus.h
// Singleton-style shared pose store. Wraps Pose2D with portENTER_CRITICAL so
// OdomService writes and any consumer reads are atomic on both cores.
#pragma once
#include <Arduino.h>
#include "OdometryLib.h"  // provides Pose2D

class PoseBus {
public:
  Pose2D get() const {
    portENTER_CRITICAL(&mux_); Pose2D p = pose_; portEXIT_CRITICAL(&mux_); return p;
  }
  void reset() {
    portENTER_CRITICAL(&mux_); pose_ = Pose2D{}; portEXIT_CRITICAL(&mux_);
  }
  void update(const Pose2D& p) {
    portENTER_CRITICAL(&mux_); pose_ = p; portEXIT_CRITICAL(&mux_);
  }
private:
  mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  Pose2D pose_{};
};
