// Transforms.hpp
// 2D rotation helper (Rot2) and deg<->rad conversion utilities used throughout the firmware.
#pragma once
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
inline float deg2rad(float d){ return d * (float)M_PI/180.0f; }
inline float rad2deg(float r){ return r * 180.0f/(float)M_PI; }
struct Rot2 {
  float c, s;
  explicit Rot2(float yaw_rad): c(cosf(yaw_rad)), s(sinf(yaw_rad)) {}
  inline void apply(float x, float y, float& xo, float& yo) const {
    xo = c*x - s*y; yo = s*x + c*y;
  }
};
