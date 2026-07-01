// LidarProjector.h
// Projects VL53L5CX grid measurements from sensor frame into world frame using the
// current PoseBus pose. Used by WiFiService to stream world-frame points to the host.
#pragma once
#include <Arduino.h>
#include "../services/LidarService.h"
#include "../core/PoseBus.h"

class LidarProjector {
public:
  LidarProjector(LidarService& svc, PoseBus& pose): svc_(svc), pose_(pose) {}

  // Top 4 rows × 8 cols of the 8×8 grid (rows 0-3, positive elevation only).
  // Fills xw/yw/zw (mm) for up to 32 zones; bit i of mask set when zone is valid.
  // Returns false if no sensor data.
  bool computeWorldPoints32(float xw[32], float yw[32], float zw[32],
                             uint32_t& mask) const;

private:
  LidarService& svc_;
  PoseBus&      pose_;
};
