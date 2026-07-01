#include "LidarProjector.h"
#include "../core/Transforms.hpp"
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Projects the top 4 rows (positive elevation) of the 8×8 grid into world-frame
// 3D points.  Row 0 = most upward, row 3 = least upward (just above horizontal).
// Rows 4-7 (negative elevation) are skipped — they look downward and hit the floor.
bool LidarProjector::computeWorldPoints32(float xw[32], float yw[32], float zw[32],
                                          uint32_t& mask) const {
  uint16_t grid[64];
  if (!svc_.getGrid(grid)) { mask = 0; return false; }

  auto P = pose_.get();
  const float yaw_robot = P.yaw_rad;
  const auto& lidar = svc_.lib();
  const auto  cal   = lidar.calibration();

  // Sensor origin offset rotated to world frame (xy only; z via z_off_mm)
  const float cry = cosf(yaw_robot), sry = sinf(yaw_robot);
  const float ox_w = cry * cal.x_off_mm - sry * cal.y_off_mm;
  const float oy_w = sry * cal.x_off_mm + cry * cal.y_off_mm;

  const float yaw_tot = yaw_robot + deg2rad(cal.yaw_off_deg);
  const float cst = cosf(yaw_tot), snt = sinf(yaw_tot);

  mask = 0;
  for (int row = 3; row <= 3; row++) {       // row 3 only: just above horizontal, closest to centre
    const float el = lidar.elevationForRowRad(row);
    if (el < 0.0f) continue;                 // safety guard

    const float cel = cosf(el), sel = sinf(el);

    for (int col = 0; col < 8; col++) {
      const int   grid_idx = row * 8 + col;  // index into 64-zone grid
      const int   out_idx  = row * 8 + col;  // output indices 0-31
      const float d = (float)grid[grid_idx];

      if (d < 50.f || d > 2500.f) {
        xw[out_idx] = 0; yw[out_idx] = 0; zw[out_idx] = 0;
        continue;
      }

      const float az = lidar.azimuthForColumnRad(col);
      const float xs = d * cel * cosf(az);   // sensor-frame: x=fwd, y=lat, z=up
      const float ys = d * cel * sinf(az);
      const float zs = d * sel;

      const float px = xs * cst - ys * snt;  // rotate horizontal to world frame
      const float py = xs * snt + ys * cst;

      const float z_world = cal.z_off_mm + zs;
      // Accept only returns at wall-like heights: 50 mm off floor up to 1 500 mm.
      // This rejects floor glints (z < 50) and ceiling hits (z > 1500).
      if (z_world < 50.f || z_world > 1500.f) {
        xw[out_idx] = 0; yw[out_idx] = 0; zw[out_idx] = 0;
        continue;
      }

      xw[out_idx] = P.x_mm + ox_w + px;
      yw[out_idx] = P.y_mm + oy_w + py;
      zw[out_idx] = z_world;
      mask |= (1u << out_idx);
    }
  }
  return mask != 0;
}
