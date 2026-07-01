# Architecture

High-level view of BugBot across firmware, simulation, and ROS 2. For the firmware service
architecture in detail, see [`../firmware/ARCHITECTURE.md`](../firmware/ARCHITECTURE.md).

## The core problem

BugBot's drive is four omni wheels in an X, each on a direct-drive motor axle. This is cheap
and simple, but open-loop it is chaotic: commanding a motion also produces parasitic
rotation and vibration, so the robot does not go where told without correction. The design
answer is sensing plus control, not precision mechanics.

## Signal flow

```
             wheel commands              motion (chaotic, open-loop)
 controller ----------------> X-omni drive ----------------> robot in world
     ^                                                            |
     |                          sensors: IMU, ToF, optical flow, AprilTag
     |                                                            |
     +------------- 3-DOF EKF pose estimate (x, y, yaw) <---------+
                     (micro-corrections close the loop)
```

The firmware runs this loop today with a hand-tuned controller and the EKF. The
[`simulation/`](../simulation) reproduces the chaotic drive and trains a policy to perform
the same corrective role, and [`ros2/`](../ros2) exposes it as a standard ROS 2 velocity
interface and bridges the policy to sim and hardware.

## The drive mix

Motors ordered (BR, FR, BL, FL); longitudinal is forward+, lateral is right+, rot is
CCW/left+:

```
d0 (BR) = +longitudinal + lateral + rot
d1 (FR) = -longitudinal + lateral - rot
d2 (BL) = +longitudinal - lateral - rot
d3 (FL) = -longitudinal - lateral + rot
```

Firmware, simulation, and the ROS 2 bridge all share this mapping.
