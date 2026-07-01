#include "WiFiGlue.h"
#include "../core/PoseBus.h"
#include "../services/LidarService.h"

static PoseBus*      s_pose  = nullptr;
static LidarService* s_lidar = nullptr;

void WiFiGlue::init(PoseBus* pose, LidarService* lidar){
  s_pose  = pose;
  s_lidar = lidar;
}

void WiFiGlue::GetPositionNonBlocking(float& x_mm, float& y_mm, float& yaw_deg){
  auto P = s_pose->get();
  x_mm   = P.x_mm;
  y_mm   = P.y_mm;
  yaw_deg= P.yaw_rad * 180.0f / 3.14159265358979323846f;
}

bool WiFiGlue::GetLidarWorldPoints(float xw[4], float yw[4], uint8_t &valid_mask, uint8_t &row_out){
  (void)xw; (void)yw;
  valid_mask = 0;
  row_out    = s_lidar ? s_lidar->row() : 0;
  return false;
}
