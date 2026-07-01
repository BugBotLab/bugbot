# BugBot

BugBot is a compact robot built for learning, prototyping, and classroom use.

- **Omnidirectional drive** — the robot can move in any direction without turning first
- **Computer vision** — the robot can see and identify objects around it
- **Depth sensing** — the robot can sense the surrounding space to build a map and avoid obstacles
- **Motion tracking** — know where the robot has been and where it is now
- **Wireless** — control the robot from your PC with no cables

All controllable from Python with a few lines of code.

```python
from bugbot import Robot

with Robot(0) as bot:
    bot.led("green")
    bot.forward(50)          # drive forward at 50% speed
    import time; time.sleep(2)
    bot.stop()
```

---

<div class="grid cards" markdown>

-   **Getting Started**

    ---

    Install the library, pair your robot, and run your first program in about ten minutes.

    [:octicons-arrow-right-24: Getting Started](getting-started.md)

-   **Lessons**

    ---

    Self-guided coding activities — run the code, see what happens, then tackle the challenges.

    [:octicons-arrow-right-24: Lessons](lessons/index.md)

-   **Hardware**

    ---

    Components, pin assignments, I2C bus, sensors, and memory layout.

    [:octicons-arrow-right-24: Hardware](hardware/index.md)

-   **Python Library**

    ---

    Full API reference for the `Robot` and `Dongle` classes, motion commands, sensors, and exceptions.

    [:octicons-arrow-right-24: Python Library](python-api/index.md)

-   **Protocol**

    ---

    ESP-NOW wire protocol, COBS framing, packet formats, and firmware internals.

    [:octicons-arrow-right-24: Protocol Reference](espnow/00-protocol.md)

</div>

---

## What's inside the box

| | |
|---|---|
| **Drive** | 4× DC motors via TB6612FNG drivers |
| **Depth** | VL53L5CX 8 × 8 ToF, 30 Hz, up to ~4 m |
| **Vision** | OV5640 camera with AprilTag detection for absolute position |
| **IMU** | BMI270 (primary) + BNO055 (9-axis), 200 Hz odometry |
| **Optical flow** | PMW3360 motion sensor breakout |
| **Wireless** | ESP-NOW over 2.4 GHz, via BugBot USB dongle |
| **CPU** | XIAO ESP32-S3 Sense — 240 MHz, 8 MB Flash, 8 MB PSRAM |
