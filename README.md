# BugBot

BugBot is a small holonomic robot that moves by **ultrasonic friction modulation**.
Instead of wheels, its four feet are vibrated at ultrasonic frequencies; modulating the
vibration modulates the effective friction between each foot and the ground, letting the
robot slide in any direction at once (forward, sideways, and turning together).
Friction-based actuation is inherently variable: effective friction shifts with surface,
load, and amplitude, so a plain "go forward" command does not travel straight. BugBot
moves accurately because a full sensor suite closes the loop and applies continuous micro
corrections.

That is the whole idea: **reliable motion out of deliberately unreliable actuation**, using
sensing and control rather than precision mechanics.

This repository holds everything for BugBot except the electronic (PCB) design: the
on-robot firmware, the host control software, a physics simulation with reinforcement
learning, ROS 2 integration, and the mechanical hardware.

## How it stays on track

BugBot fuses four sensors to estimate where it really is and correct in real time:

- **IMU** (Bosch BMI270) for orientation and angular rate,
- **Time-of-flight** ranging (ST VL53L5CX) for walls and obstacles,
- **Optical flow** (PMW3360) for ground-relative velocity,
- **AprilTag** vision for absolute position,

all combined in a 3-DOF (x, y, yaw) Extended Kalman Filter. The controller reads that
estimate and nudges the four feet many times a second to hold the commanded path.

## Vision and edge AI

The on-board camera does more than stream video. On the robot it provides AprilTag
detection for absolute localisation; heavier computer vision runs at the edge, in the
operator's browser (the web UI ships OpenCV.js face detection as a worked example), so
students can build AI behaviours on top of BugBot using the compute they already have,
with no extra hardware.

## Repository layout

| Folder | Contents |
| --- | --- |
| `firmware/` | On-robot firmware (ESP-IDF). The friction-modulation drive mix (`MotionLib`), sensor drivers, the 3-DOF EKF pose service, runtime calibration, and an on-board web UI. |
| `software/` | Host-side control. `web-ui/` is a browser control panel (WebSocket telemetry, camera view, OpenCV.js face detection). |
| `simulation/` | Physics sim + reinforcement learning in NVIDIA Isaac Lab. Reproduces the chaotic friction-modulated drive and trains a corrective policy. *(In progress.)* |
| `ros2/` | ROS 2 integration: a node that maps `/cmd_vel` (full Twist) to the four foot drive commands and bridges the trained policy to the robot. *(In progress.)* |
| `hardware/` | Mechanical only: CAD (STEP/STL), assembly guide, and BOM. Electronics/PCB design is intentionally not included. *(To be added.)* |
| `docs/` | Architecture notes and the drive/sensing rationale. |

## The simulation, in short

The simulation does not idealise the robot. It drives the four feet through BugBot's
exact drive mix and injects the disturbances that make the real motion chaotic: per-foot
friction variation, vibration and coupling between translation and rotation, actuation
latency, and mass and centre-of-gravity jitter. A reinforcement-learning policy observes
the same signals the real robot senses and learns the micro corrections needed to track a
commanded holonomic velocity. The trained policy deploys back to the physical robot over
ROS 2.

## Firmware quick start

Requires ESP-IDF v5.x.

```bash
cd firmware
# Set your Wi-Fi in main/lib/config/WiFiConfig.h (placeholders by default),
# or leave blank and configure at runtime via the AP config portal.
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor
```

See [`firmware/README.md`](firmware/README.md) and [`firmware/ARCHITECTURE.md`](firmware/ARCHITECTURE.md)
for the service architecture, the drive mix, and the pose EKF.

## Web control UI

Open `software/web-ui/index.html` (served over HTTP) and point it at the robot's address
(defaults to the BugBot access point at `192.168.4.1`).

## Status

BugBot is an active BugBotLab project. Firmware and the web UI are migrated and working;
the simulation and ROS 2 integration are under construction; mechanical CAD is being added.

## License

MIT, see [`LICENSE`](LICENSE). Hardware design files, when added, are provided for
reference. The electronic (PCB) design is not part of this repository.
