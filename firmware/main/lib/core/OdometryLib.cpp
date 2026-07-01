#include "OdometryLib.h"


void Odometry2D::integrate(float dxb_mm, float dyb_mm, float abs_yaw_deg) {
const float c = cosf(_pose.yaw_rad), s = sinf(_pose.yaw_rad);
_pose.x_mm += c * dxb_mm - s * dyb_mm;
_pose.y_mm += s * dxb_mm + c * dyb_mm;
_pose.yaw_rad = abs_yaw_deg *(M_PI/180.0f);
if (_pose.yaw_rad > M_PI) _pose.yaw_rad -= 2.0f * M_PI;
if (_pose.yaw_rad < -M_PI) _pose.yaw_rad += 2.0f * M_PI;
}


void Odometry2D::getPose(float& x_mm, float& y_mm, float& yaw_deg) const {
x_mm = _pose.x_mm; y_mm = _pose.y_mm; yaw_deg = _pose.yaw_rad * 180.0f / M_PI;
}