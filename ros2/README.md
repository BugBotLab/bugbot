# ROS 2 integration

ROS 2 packages for BugBot. Status: **in progress.**

## Goal

Bridge BugBot's holonomic drive and its sensor/pose data into ROS 2, and run the trained
corrective policy from [`../simulation/`](../simulation) both in simulation and on the real
robot.

## Planned

- A node that subscribes to `/cmd_vel` (full `geometry_msgs/Twist`, because lateral motion
  is real for a holonomic robot) and maps the commanded twist to the four wheel commands
  using the same X-omni mix as the firmware and simulation.
- Sensor and odometry topics (IMU, range, pose/odom) published from the robot or the sim.
- A policy node that loads a trained policy and publishes wheel commands, with a shared
  deploy core reused between the sim bridge and the real-robot bridge (UDP/serial).
- The Isaac ROS 2 bridge for closed-loop validation in simulation.

## Platform

ROS 2 (Humble/Jazzy) on Ubuntu or WSL 2. Package type: `ament_python`.
