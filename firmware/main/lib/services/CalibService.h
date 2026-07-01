#pragma once
#include <Arduino.h>
#include "../core/KinematicModel.h"
#include "../core/PoseBus.h"
#include "../net/WebWS.h"

class MotionService;

// State-machine driven kinematic calibration.
// tick() must be called from loop() — it is non-blocking.
// During a run it takes exclusive control of the motors; the caller must guard
// setCommandVec() with isRunning() to prevent teleop interference.
class CalibService {
public:
  bool begin(MotionService& motion, PoseBus& bus, WebWS& ws);

  // Load previously saved model from /config/kinematics.cfg on FFat.
  // Returns true if a valid calibrated model was found.
  bool loadFromFlash();

  void trigger();
  void cancel();

  // Called by AprilTagService from Core 0. Thread-safe.
  void onAprilTags(uint8_t n, const WebWS::AprilTagHit* hits);

  // Advance the state machine. Call from loop() every cycle.
  void tick();

  bool isRunning() const;
  const KinematicModel& model() const { return model_; }

private:
  enum Phase : uint8_t {
    IDLE = 0,
    WAIT_TAG,
    FWD_PRE,  FWD_MOVE,  FWD_POST,
    FWD_BACK, FWD_BACK_SETTLE,
    ROT_PRE,  ROT_CW,    ROT_POST,
    ROT_BACK, ROT_BACK_SETTLE,
    LAT_PRE,  LAT_MOVE,  LAT_POST,
    LAT_BACK, LAT_BACK_SETTLE,
    FIT_SAVE,
    DONE,
    CALIB_ERROR
  };

  struct TagAccum {
    float dist_sum = 0, az_sum = 0;
    int   n = 0;
    void  reset() { dist_sum = az_sum = 0; n = 0; }
    float dist()  const { return n > 0 ? dist_sum / n : 0.0f; }
    float az()    const { return n > 0 ? az_sum   / n : 0.0f; }
    bool  ok()    const { return n >= 1; }  // at least one detection
  };

  // Persists across phase boundaries — fallback when a settle phase gets 0 detections.
  struct CachedTag { float az_deg = 0, dist_mm = 0; uint32_t t_ms = 0; bool valid = false; };

  struct PendingTag { uint8_t id; float az_deg, dist_mm; bool fresh; };

  void advance_(Phase next, uint32_t now);
  void error_(const char* msg);
  void sendStatus_(int progress, const char* phase = nullptr, const char* err = nullptr);
  bool saveToFlash_();
  // Returns best available tag reading (accumulator > cached, max 3s stale).
  bool bestTag_(float& dist, float& az, uint32_t now) const;
  // Send a body-frame motion command, pre-rotating to cancel MotionService's
  // field-centric transform.  Rotation is field-centric-agnostic; pass through.
  void setBodyCmd_(float fwd, float lat, float rot);

  MotionService* motion_ = nullptr;
  PoseBus*       bus_    = nullptr;
  WebWS*         ws_     = nullptr;

  Phase    phase_      = IDLE;
  uint32_t phaseStart_ = 0;
  TagAccum tagAccum_;

  float    distBefore_     = 0.0f;  // AprilTag dist before forward move
  float    distAfter_      = 0.0f;  // AprilTag dist after  forward move
  float    yawBefore_      = 0.0f;  // IMU yaw before rotation
  float    yawAfter_       = 0.0f;  // IMU yaw after  rotation (post-settle)
  float    latAzBefore_    = 0.0f;  // tag azimuth before lateral move
  float    latAzAfter_     = 0.0f;  // tag azimuth after  lateral move
  float    latDistAvg_     = 0.0f;  // avg distance to tag during lateral phase
  float    moveTargetYaw_  = 0.0f;  // heading to maintain during forward/lateral moves
  uint32_t rotMoveMs_      = 0;     // actual time spent in the ROT_CW phase

  portMUX_TYPE tagMux_ = portMUX_INITIALIZER_UNLOCKED;
  PendingTag   pendingTag_{};
  CachedTag    lastTag_{};

  KinematicModel model_{};

  static constexpr float    kPower        = 0.35f;   // ~6-9 cm; enough to overcome stiction
  static constexpr uint32_t kMoveMs       = 400;     // forward/back duration
  static constexpr uint32_t kLatMoveMs    = 300;     // lateral duration
  static constexpr uint32_t kSettleMs     = 600;
  static constexpr uint32_t kWaitMs       = 4000;
  static constexpr uint32_t kRotTimeoutMs = 5000;    // safety timeout for angle-based rotation
  static constexpr float    kTargetRotRad = 0.7854f; // 45° target rotation (π/4)
  static constexpr float    kHeadingKp    = 1.2f;    // proportional gain for heading correction

  static float wrapPi_(float a) {
    while (a >  3.14159265f) a -= 6.28318530f;
    while (a < -3.14159265f) a += 6.28318530f;
    return a;
  }
};
