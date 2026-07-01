# BugBot Firmware

Arduino firmware for the **XIAO ESP32-S3 Sense** autonomous robot platform.

## What it does

- Omnidirectional drive via mecanum wheels (PCA9685 → 4× TB6612 motor drivers)
- IMU dead-reckoning @ 200 Hz (Bosch BMI270 gyroscope)
- LiDAR sensing @ 15 Hz (VL53L5CX 8×8 ToF)
- AprilTag localisation (OV5640 5 MP camera, QQVGA)
- 3-DOF Extended Kalman Filter fusing IMU + ToF + AprilTag fixes
- Binary WebSocket telemetry (pose, LiDAR, occupancy map) to a Python host
- Browser-based control UI served directly from LittleFS
- Runtime configuration portal at `http://<ip>/config`
- Apple MIDI (RTP-MIDI) over Wi-Fi for buzzer/LED/motion control
- Deep-sleep mode triggered by physical toggle switch

## Build requirements

- **Arduino IDE 2.x** — board: *XIAO ESP32S3* (Espressif ESP32 package)
- **Libraries** (install via Library Manager):
  - ESPAsyncWebServer + AsyncTCP
  - Adafruit PWM Servo Driver
  - Adafruit NeoPixel
  - SparkFun VL53L5CX
  - ESP32Servo
  - AppleMIDI + MIDI Library (FortySevenEffects)
  - apriltag (ESP32 port)

## First-time setup

1. Edit `lib/config/WiFiConfig.h` — enter your Wi-Fi credentials.
2. Upload the LittleFS filesystem image (`Data/`) before or after flashing:
   ```
   # Requires littlefs upload tool for Arduino IDE
   Tools → ESP32 LittleFS Data Upload
   ```
3. Flash the firmware. On first boot, if no config files exist, defaults are written to `/config/`.
4. Connect to the web UI: `http://bugbot.local/config` (or use the IP shown on Serial).

## Recovery AP

If Wi-Fi configuration is lost or boot fails, the robot starts a recovery access point:
- **SSID:** `bugbot_setup`
- **Password:** `bugbot123`
- Then browse to `http://192.168.4.1/config`

## Apple MIDI mapping

Requires [rtpMIDI](https://www.tobias-erichsen.de/software/rtpmidi.html) on Windows or any RTP-MIDI host.

| Channel | Control | Effect |
|---|---|---|
| 1 | Note On/Off | Buzzer on/off; velocity → duty cycle |
| 1 | Pitch Bend | ±2 semitones frequency shift |
| 2 | CC 20/21/22 | Longitudinal / lateral / rotation drive |
| 3 | CC 30 | LED brightness |
| 3 | CC 31/32/33 | LED R / G / B |
| 4 | CC 40/41 | Servo 1 / Servo 2 position |

Motion from MIDI times out after 350 ms if no new CC message arrives.

## Flashing LittleFS filesystem

To flash the web UI to the robot (preserves existing /config credentials):
```powershell
# Windows — auto-detects COM port
.\tools\flash_data.ps1

# or specify port explicitly
.\tools\flash_data.ps1 -Port COM15
```

## Project layout

```
BugBot_Firmware.ino         main sketch
lib/
  config/                   hardware pin config + runtime config structs
  core/                     shared data types (Pose2D, PoseBus, kinematics)
  drivers/                  hardware drivers (BMI270, LiDAR, camera, motors)
  services/                 FreeRTOS task services
  net/                      AsyncWebServer + WebSocket facade
  util/                     task helpers, atomic flags
Data/                       web UI (served from LittleFS)
config/                     default runtime config files (motion.cfg, wifi.cfg, system.cfg)
tools/
  flash_data.ps1            flash Data/ to robot, preserving /config credentials
  make_ffat.py              build a FAT16 filesystem image from a source directory
  extract_config.py         extract /config files from a filesystem image
compile_libs_async.cpp      Arduino IDE compile-order shim (see comment inside)
compile_libs_camera.cpp     Arduino IDE compile-order shim (see comment inside)
partitions.csv              ESP32 partition table
```

See `ARCHITECTURE.md` for full data-flow diagrams and thread-safety details.
