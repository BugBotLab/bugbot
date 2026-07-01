# Simulation (NVIDIA Isaac Lab)

A physics simulation and reinforcement-learning environment for BugBot, built on NVIDIA
Isaac Sim + Isaac Lab. Status: **in progress.**

## Goal

Reproduce the *hard* part of BugBot rather than an idealised robot. The environment drives
four omni wheels through BugBot's exact X-mix and injects the disturbances that make the
real motion chaotic, then trains a policy that uses sensor feedback to correct for them.

## Planned layout

```
simulation/
├── robot/       # BugBot USD/URDF: chassis + 4 omni wheels at 45 degrees (X), meshes
├── tasks/       # Isaac Lab task: holonomic velocity/pose tracking
│                #   - action: 4 wheel speeds (via the X-omni mix)
│                #   - chaos model: per-wheel slip/friction, vibration, translation-rotation
│                #     coupling, actuation latency, mass/CoG jitter (heavy domain randomization)
│                #   - observations: the real sensor suite (IMU, range, optical-flow velocity) + command
│                #   - reward: tracking accuracy, penalise drift and jerk
└── learning/    # PPO train / play / eval scripts; runs/ outputs (gitignored)
```

## The drive mix (must match firmware)

From `firmware/main/lib/drivers/MotionLib.cpp`, motors ordered (BR, FR, BL, FL):

```
d0 (BR) = +longitudinal + lateral + rot
d1 (FR) = -longitudinal + lateral - rot
d2 (BL) = +longitudinal - lateral - rot
d3 (FL) = -longitudinal - lateral + rot
```

The simulation and the ROS 2 bridge both use this exact mapping so behaviour matches the
real robot.

## Requirements (to be finalised)

NVIDIA Isaac Sim + Isaac Lab (RTX GPU). Training runs on Windows; the ROS 2 bridge in
[`../ros2/`](../ros2) targets Ubuntu / WSL 2.
