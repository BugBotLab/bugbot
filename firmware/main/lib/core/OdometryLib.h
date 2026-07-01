// OdometryLib.h
// Pose2D data type (x_mm, y_mm, yaw_rad) and Odometry2D integrator.
// Odometry2D accumulates body-frame velocity deltas into a world-frame pose.
#pragma once
#include <Arduino.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


// ========== Pose ==========
struct Pose2D {
float x_mm = 0.0f; // world X (forward +)
float y_mm = 0.0f; // world Y (left +)
float yaw_rad= 0.0f; // CCW+ radians
};


// ========== Odometry2D ==========
// Integrates body-frame deltas into a world-frame pose.
class Odometry2D {
public:
void reset() { _pose = Pose2D{}; }


// Integrate one step (mm, mm, rad)
void integrate(float dxb_mm, float dyb_mm, float dyaw_rad);


void getPose(float& x_mm, float& y_mm, float& yaw_deg) const;


const Pose2D& pose() const { return _pose; }


private:
Pose2D _pose;
};