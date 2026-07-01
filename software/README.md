# Software

Host-side software for BugBot.

## `web-ui/`

A browser-based control panel for BugBot. It connects to the robot over WebSocket for
telemetry and control, shows the camera stream, and runs OpenCV.js in the browser
(including a Haar-cascade face detector). No build step: open `index.html` over HTTP.

Point it at the robot's address; it defaults to the BugBot access point (`192.168.4.1`).
The camera and WebSocket hosts can be changed in the UI or in `js/shared/utils.js`.

## Planned

Host-side Python control and the sim-to-real deployment bridge (loads a trained policy and
drives the robot over UDP/ROS 2) will live here alongside the web UI. See
[`../ros2/`](../ros2) and [`../simulation/`](../simulation).
