// WiFiGlue.h
// Adapter bridging the legacy WiFiLib callback interface to live PoseBus /
// LidarService state.
#pragma once
#include <Arduino.h>

class PoseBus;
class LidarService;

namespace WiFiGlue {
  void init(PoseBus* pose, LidarService* lidar);

  void GetPositionNonBlocking(float& x_mm, float& y_mm, float& yaw_deg);

  // LidarProjector removed — always returns false
  bool GetLidarWorldPoints(float xw[4], float yw[4], uint8_t &valid_mask, uint8_t &row_out);
}
