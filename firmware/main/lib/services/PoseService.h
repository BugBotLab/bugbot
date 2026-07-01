#pragma once
#include <Arduino.h>
#include <functional>
#include "../core/PoseBus.h"
#include "../core/KinematicModel.h"
#include "../net/WebWS.h"
#include "../util/TaskUtil.hpp"
#include "LidarService.h"

class MotionService;
class OdomService;

// 3-DOF EKF fusing IMU yaw, LiDAR scan-flow odometry, LiDAR scan-to-map wall matching,
// and AprilTag absolute fixes.
//
// State: [x_mm, y_mm, yaw_rad]
// Covariance: symmetric 3x3, stored upper-triangle (p00,p01,p02,p11,p12,p22)
//
// Position source priority:
//   AprilTag       -- absolute fix: distance circles (yaw-independent) + azimuth yaw
//   LiDAR scan-map -- up to 24 beams per scan → full [x,y,yaw] Jacobian; corrects
//                     heading even without tags via wall geometry
//   IMU            -- high-rate yaw measurement (EKF scalar update each cycle)
//   ToF scan-flow  -- relative (dx,dy) odometry; primary PREDICTION model
//   Kinematics     -- fallback prediction when ToF is unavailable
//
// Startup: EKF begins with large position variance so scan-to-map can converge from
// any starting position without requiring a tag to be visible at boot.
class PoseService {
public:
  bool begin(PoseBus& bus, MotionService& motion, const KinematicModel& model,
             LidarService* lidar = nullptr, OdomService* odom = nullptr);

  // Called by AprilTagService on Core 0 -- thread-safe.
  void onAprilTags(uint8_t n, const WebWS::AprilTagHit* hits);

  // Reset position to origin with tight covariance (redefines world frame).
  void resetPose();
  // Forget all learned tag world positions (keeps pre-defined tags intact if called
  // from defineTag; clears everything if called from UI reset).
  void resetMap();

  // Pre-define a tag's world position so it is used as an absolute anchor immediately,
  // without waiting for the online learning phase.
  // face_deg: direction the tag surface faces, degrees CCW from +X axis
  //   (0=toward +X, 90=toward +Y/left, 180=toward -X, 270=toward -Y/right).
  // Thread-safe: may be called from the WebSocket handler (Core 0).
  void defineTag(uint8_t id, float x_mm, float y_mm, float face_deg);

  // Define all 8 arena tags from physical dimensions.
  // Origin = bottom-left inner corner, +X right, +Y up.
  // 2 tags per wall at 1/3 and 2/3, clockwise from top-left: 0,1=top 2,3=right 4,5=bottom 6,7=left
  void applyArenaLayout(float widthMm, float heightMm, float cornerMm);

  // ── Localization sweep ────────────────────────────────────────────────────────
  // Rotates the robot slowly through a full 360° while the scan-to-map and AprilTag
  // EKF updates converge absolute position and heading.  Stops early if yaw covariance
  // tightens below threshold before completing a full rotation.
  //
  // onLocSweepComplete is called from the PoseService task (Core 1) when the sweep
  // finishes; cb(true) = converged early, cb(false) = completed full rotation.
  void startLocalizationSweep();
  void stopLocalizationSweep();
  bool isLocSweepActive() const { return locSweepActive_; }
  void onLocSweepComplete(std::function<void(bool)> cb) { sweepDoneCb_ = std::move(cb); }

private:
  static void taskThunk(void* arg) { static_cast<PoseService*>(arg)->run(); }
  void run();

  struct TagObs   { uint8_t id; float az_deg, dist_mm, tag_yaw_deg; uint32_t t_ms; bool fresh; };
  struct TagWorld { float x_mm, y_mm, face_yaw_rad; bool known; };

  // EKF primitives
  void predict_(float dx_w, float dy_w, float dyaw, float q_pos, float q_yaw);
  // Single scalar measurement update: z = h0*x + h1*y + h2*yaw + noise(R)
  void updateScalar_(float h0, float h1, float h2, float innov, float R);

  // Returns false on first sighting (learns the tag) or if geometry is unreliable.
  bool getTagObs_(const TagObs& obs, float& z_x, float& z_y, float& z_yaw,
                  float& r_pos, float& r_yaw);

  // Returns world-frame displacement for use in predict_(); false = bad scan.
  bool computeToFOdometry_(float& dx_w, float& dy_w);

  // Scan-to-map EKF updates: ray-casts each near-horizontal LiDAR beam against the
  // rectangular arena model and applies a full [x,y,yaw] Jacobian EKF update.
  // Replaces the old applyToFWallUpdates_() which only produced scalar x or y updates.
  void applyToFScanMatchUpdate_();

  // Ray-casts a single beam from (x,y) in direction dir_rad against the rectangular
  // arena (0..arenaW_, 0..arenaH_).  Returns NAN if outside arena or no hit within
  // 6000 mm.  Sets *hitXWall=true if the hit wall is a constant-x wall (left/right).
  float rayCastRect_(float x, float y, float dir_rad, bool* hitXWall) const;

  PoseBus*              bus_    = nullptr;
  MotionService*        motion_ = nullptr;
  const KinematicModel* model_  = nullptr;
  LidarService*         lidar_  = nullptr;
  OdomService*          odom_   = nullptr;
  TaskHandle_t          th_     = nullptr;

  portMUX_TYPE tagMux_ = portMUX_INITIALIZER_UNLOCKED; // guards latestTags_
  TagObs   latestTags_[4] = {};
  uint8_t  nLatestTags_   = 0;

  portMUX_TYPE mapMux_ = portMUX_INITIALIZER_UNLOCKED; // guards tagMap_ + nKnownTags_
  uint8_t  nKnownTags_    = 0;
  TagWorld tagMap_[16]    = {};

  // EKF state: x (mm), y (mm), yaw (rad)
  float x_   = 0.0f, y_ = 0.0f, yaw_ = 0.0f;
  // Symmetric 3x3 covariance, upper triangle.
  // Start with large position variance so scan-to-map can converge from any starting
  // position without an AprilTag.  resetPose() sets tight variance at a known origin.
  float p00_ = 200000.0f, p01_ = 0.0f, p02_ = 0.0f;
  float                   p11_ = 200000.0f, p12_ = 0.0f;
  float                                     p22_ = 1.0f;

  float arenaW_ = 0.0f;
  float arenaH_ = 0.0f;
  uint32_t lastScanMatchGen_ = 0xFFFFFFFF;  // LiDAR generation last used for scan-match

  // ── Tuning constants ──────────────────────────────────────────────────────
  // Process noise rates (per second; multiplied by dt each step)
  static constexpr float kQPosToF  = 100.0f;   // mm^2/s -- ToF prediction residual
  static constexpr float kQPosCmd  = 2000.0f;  // mm^2/s -- kinematic fallback
  static constexpr float kQYaw     = 0.0002f;  // rad^2/s -- IMU drift (ToF path)
  static constexpr float kQYawCmd  = 0.001f;   // rad^2/s -- kinematic yaw uncertainty

  // Measurement noise variances
  static constexpr float kRImuYaw     = 0.01f;   // original tight value (kept for reference)
  // Weak IMU fallback: ~10× looser than tag face-direction so tags dominate whenever visible.
  static constexpr float kRImuYawWeak = 0.5f;
  static constexpr float kRTagPosBase = 200.0f;   // mm^2   -- close, frontal (sigma~14mm)
  static constexpr float kRTagPosDist = 0.002f;   // mm^2 per mm^2 of distance squared
  static constexpr float kRTagPosAng  = 1000.0f;  // mm^2 per rad^2 of oblique angle
  static constexpr float kRTagYawBase = 0.005f;   // rad^2  -- close, frontal (~4 deg)
  static constexpr float kRTagYawDist = 2.0e-6f;  // rad^2 per mm^2 of distance squared
  static constexpr float kRTagYawAng  = 0.4f;     // rad^2 per rad^2 of oblique angle

  // Position covariance set after a tag fix (mm^2); sigma ~14mm at close range
  static constexpr float kPosAfterTagFix = kRTagPosBase;
  // Soft arena-wall noise (mm^2); tighter than tag noise so walls win over bad tag fixes
  static constexpr float kRArenaWall     = 100.0f;
  // Only learn a new tag when position variance is below this; first tag exempt
  static constexpr float kTagLearnMaxP = 200000.0f; // sigma~447mm
  // Minimum ToF beams for a valid scan-flow estimate
  static constexpr int   kMinToFBeams  = 3;
  // Scan-to-map: measurement noise for rayCast-based wall range updates (mm²).
  // Softer than tag fixes; distance-dependent scaling keeps tangent beams well-damped.
  static constexpr float kRScanBase        = 1600.0f;  // close, perpendicular: σ=40mm
  static constexpr float kRScanDist        = 0.003f;   // mm² per mm² of range²
  // Innovation gate for scan-to-map (mm).  Wide gate used while position variance
  // is large (startup / kidnap) so the EKF can converge from an unknown position.
  static constexpr float kScanMatchGateMm      = 200.0f;
  static constexpr float kScanMatchGateWideMm  = 500.0f;
  // Minimum incidence cosine for a beam to contribute a yaw Jacobian term.
  // Beams more than ~60° from wall-normal produce numerically large h2 terms; skip them.
  static constexpr float kScanYawMinCos    = 0.5f;
  // Scan residual threshold for anchoring the IMU to wall-derived heading (mm).
  static constexpr float kScanHeadingLockMm    = 30.0f;
  static constexpr int   kScanHeadingMinBeams  = 8;

  // Adaptive process noise: Q scales with command magnitude so uncertainty grows
  // faster when moving and is nearly frozen when stationary.
  static constexpr float kQPosScale    = 3.0f;   // multiplier per unit translation cmd
  static constexpr float kQYawScale    = 5.0f;   // multiplier per unit rotation cmd

  // Zero-velocity update (ZUPT): thresholds for declaring the robot stationary.
  static constexpr float kZuptAccelG   = 0.08f;  // g-units horizontal accel magnitude
  static constexpr float kZuptGyroDps  = 0.25f;  // dps, matches existing bias-tier 1

  // ── Localization sweep state ──────────────────────────────────────────────────
  std::function<void(bool)> sweepDoneCb_{};
  volatile bool locSweepActive_ = false;
  float sweepAccumYaw_ = 0.0f;  // total rotation accumulated since sweep start (rad)
  float sweepLastYaw_  = 0.0f;  // yaw_ at previous EKF tick, for delta tracking
  bool  sweepTagSeen_  = false;  // true if ≥1 tag update fired during this sweep

  // Running mean residual from the last applyToFScanMatchUpdate_() call (mm).
  // Used to verify the EKF actually converged to the correct pose, not a local minimum.
  float lastScanMeanResid_ = 999.f;
  int   lastScanBeams_     = 0;

  // All-beam mean residual threshold for trusted convergence (mm).
  // Using all attempted beams (not just gate-passers) so wrong attractors —
  // where most beams are rejected — produce high means and fail this check.
  static constexpr float kSweepResidOkMm  = 80.0f;   // all-beam mean (higher than gate-only)
  static constexpr int   kSweepResidMinB  = 6;        // gate-passing beams still needed

  // Rotation command sent to MotionService during sweep (normalized, 0–1).
  // ~0.12 gives roughly 1 rad/s actual rate, completing a full 360° in ~6-7 s
  // and giving the 15 Hz LiDAR ~90 scans per revolution to converge position.
  static constexpr float kLocSweepRotSpeed = 0.12f;
  // Yaw covariance threshold for early-stop (rad²); σ ≈ 5° means heading is known well
  static constexpr float kSweepDoneP22     = 0.008f;
  // Also require position to be reasonably converged before declaring early-stop
  static constexpr float kSweepDonePos     = 15000.0f;  // mm² — σ ≈ 122 mm
  // Minimum rotation before any early-stop is considered (rad).
  // Set close to a full circle so the sweep always covers the whole arena; early-stop
  // only fires in the last 30° if the EKF happened to converge from an AprilTag.
  static constexpr float kSweepMinYaw      = 2.0f * (float)M_PI - 0.5f;  // ~309°
  // Hard stop after this much rotation (rad)
  static constexpr float kSweepMaxYaw      = 2.0f * (float)M_PI + 0.5f;  // ~400°

  uint32_t ekfStableMs_   = 0;   // when EKF first became "converged" in current window

  // IMU dead-reckoning: previous yaw reading for delta computation
  float prevImuYaw_      = 0.0f;
  bool  prevImuYawValid_ = false;

  // Reference scan for ToF range-flow odometry
  struct ToFScan {
    uint16_t ranges[64] = {};
    float    yaw_rad    = 0.0f;
    uint32_t gen        = 0xFFFFFFFF;
    bool     valid      = false;
  } prevScan_;
};
