#pragma once

// Linear dead-reckoning model for the omni-drive robot.
// All three axes are independent: the motor mix is symmetric so fwd and lat
// could differ slightly due to roller geometry, so they are calibrated separately.
struct KinematicModel {
  float k_lin_fwd_mm_s = 0.0f;  // mm/s of body-forward travel per unit longitudinal cmd
  float k_lin_lat_mm_s = 0.0f;  // mm/s of body-lateral  travel per unit lateral cmd
  float k_rot_rad_s    = 0.0f;  // rad/s of yaw rate per unit rotational cmd
  bool  calibrated      = false;
};
