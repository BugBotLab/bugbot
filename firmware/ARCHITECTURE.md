# Firmware Architecture

BugBot firmware runs on the **XIAO ESP32-S3 Sense** under the Arduino + FreeRTOS framework.
All major subsystems run as independent FreeRTOS tasks. This document explains how they
fit together so you can navigate the code without reading every file first.

---

## Layer Map

```
BugBot_Firmware.ino          ← entry point; owns all objects, wires them together, starts tasks
│
├── lib/config/              ← compile-time and runtime configuration
│   ├── RobotConfig.h        ← pin assignments, polling frequencies (static, edit for hardware changes)
│   ├── AppConfig.h          ← runtime tunable structs (loaded from LittleFS by ConfigService)
│   └── WiFiConfig.h         ← compile-time Wi-Fi credentials (fallback / legacy UDP path)
│
├── lib/core/                ← shared data types and cross-cutting utilities
│   ├── OdometryLib.h/cpp    ← Pose2D type; Odometry2D integrates body-frame deltas
│   ├── PoseBus.h            ← thread-safe Pose2D shared store (critical section)
│   ├── KinematicModel.h     ← calibration constants for dead-reckoning prediction
│   ├── DriveDefs.hpp        ← DriveDir enum (shared by firmware and Python host)
│   ├── Transforms.hpp       ← 2D rotation (Rot2), deg↔rad helpers
│   └── BusLocks.hpp         ← I2C_LOCK / I2C_UNLOCK wrappers for g_i2cMutex
│
├── lib/drivers/             ← hardware abstraction (no FreeRTOS tasks)
│   ├── BMI270Driver         ← gyro: raw Z-rate → integrated yaw with bias correction
│   ├── LidarLib             ← VL53L5CX: 8×8 ToF grid, calibration, azimuth LUT
│   ├── CameraDriver         ← OV5640: frame grab/release, resolution/quality setters
│   ├── MotionLib            ← PCA9685: 4-motor mecanum PWM + direction logic
│   ├── I2CBus               ← Wire wrapper with FreeRTOS mutex (withLock())
│   ├── BoardPowerLib        ← enable pin, USB detect, battery ADC
│   ├── RgbLedLib            ← WS2812 NeoPixel (GRB corrected)
│   ├── BuzzerLib            ← ledc PWM tone / off
│   ├── ServoLib             ← dual PWM servo (ESP32Servo)
│   ├── LedLib               ← simple GPIO LED on/off
│   ├── SDCardDriver         ← SD card mount / file access (optional)
│   └── GroundSensorsLib     ← optical flow sensors on SPI (PMW3901, experimental)
│
├── lib/services/            ← FreeRTOS task services (one class = one task)
│   ├── OdomService          ← BMI270 @ 200 Hz → Odometry2D → PoseBus
│   ├── MotionService        ← drive commands → MotionLib PWM; heading-hold PID
│   ├── LidarService         ← VL53L5CX poll → double-buffered 8×8 grid
│   ├── LidarProjector       ← grid → world-frame 3D points (uses PoseBus)
│   ├── CameraService        ← esp-idf HTTP server: /capture, /stream, /set (port 81)
│   ├── CameraAPI            ← thin C-function wrapper around CameraService
│   ├── AprilTagService      ← JPEG → tag detection → TagCallback + WebSocket
│   ├── PoseService          ← 3-DOF EKF: IMU yaw + ToF flow + AprilTag fixes
│   ├── WiFiService          ← Wi-Fi bring-up (STA/AP) + WebWS + telemetry TX task
│   ├── ConfigService        ← LittleFS key-value config load/save
│   ├── ConfigPortalService  ← HTTP /config portal (attach to WebWS)
│   ├── CalibService         ← state-machine kinematic calibration (non-blocking tick)
│   ├── ActuatorService      ← servo + buzzer + LED unified; WebSocket JSON handler
│   ├── DroidChime           ← non-blocking audio/LED event cues (FreeRTOS queue)
│   ├── DiagService          ← serial diagnostics over FreeRTOS task
│   ├── MidiService          ← Apple MIDI → buzzer/LED/motion (called from loop())
│   ├── OccupancyMap         ← 64×64 log-odds grid, Bresenham ray-casting
│   ├── WiFiLib              ← legacy UDP telemetry sender (raw PosePkt/LidarPkt)
│   └── WiFiGlue             ← adapter: WiFiLib callbacks → PoseBus / LidarProjector
│
├── lib/net/
│   └── WebWS                ← AsyncWebServer (port 80) + WebSocket /ws; all send*() helpers
│
└── lib/util/
    ├── TaskUtil.hpp         ← CreateTaskPinned/PSRAM, Periodic() loop helper
    └── AtomicFlags.hpp      ← AtomicFlag / AtomicBits (std::atomic wrappers) + Flags:: constants
```

---

## Thread Safety Rules

| Shared Resource | Protection Mechanism | Who Writes | Who Reads |
|---|---|---|---|
| `PoseBus` (Pose2D) | `portENTER_CRITICAL` | OdomService, PoseService | MotionService, LidarProjector, WiFiService TX task, DiagService |
| I2C bus (Wire) | `g_i2cMutex` (semaphore) via `I2C_LOCK/UNLOCK` | BMI270Driver, LidarLib | — |
| `LidarService` grid | `portENTER_CRITICAL` (`mux_`) | LidarService task | LidarProjector, PoseService |
| `CalibService` tag obs | `portMUX_TYPE` (`tagMux_`) | AprilTagService (Core 0) | CalibService `tick()` (loop) |
| `PoseService` tag obs | `portMUX_TYPE` (`tagMux_`) | AprilTagService (Core 0) | PoseService task |
| WebSocket send | `SemaphoreHandle_t sendMtx_` in WebWS | WiFiService TX task, loop() | — |

---

## Key Data Flows

### 1. Odometry
```
BMI270 (I2C, Core 0)
  → BMI270Driver::readYawDeg()
  → OdomService task (200 Hz, Core 1)
  → Odometry2D::integrate()
  → PoseBus::update()
```

### 2. AprilTag Localisation
```
OV5640 (PSRAM frame)
  → AprilTagService task (Core 0, ~5 Hz)
  → apriltag_detector_detect()
  → TagCallback → PoseService::onAprilTags()
                → CalibService::onAprilTags()
  → WebWS::sendAprilTags()  (→ host)
  → OdomService::correctYawRad()  (yaw anchor)
```

### 3. LiDAR → World → Host
```
VL53L5CX (I2C)
  → LidarLib::getGrid()
  → LidarService task (15 Hz, Core 1) → double-buffer grid
  → LidarProjector::computeWorldPoints32()  (uses PoseBus pose)
  → WiFiService TX task → WebWS::sendScan3D()  (→ host)
  → OccupancyMap::markRay()  (updated in TX task)
  → WebWS::sendMap() / sendMapDelta()  (→ host)
```

### 4. Control (Host → Motors)
```
Host WebSocket frame (binary, magic 0x4242)
  → WebWS onControl / onMotionVec callback
  → MotionService::setCommandVec()
  → MotionService task → heading-hold PID → MotionLib::drive()
  → PCA9685 (I2C) → 4 × TB6612 motor drivers
```

### 5. Pose EKF (PoseService)
```
Each cycle (~50 Hz):
  ToF scan-flow (LidarService grid diff)  ─┐
  Kinematic prediction (MotionService cmd) ─┤→ EKF predict_()
  IMU yaw (PoseBus)                       ─┘

  AprilTag hit (onAprilTags, cross-core)  ──→ EKF updateScalar_() + tag map learning

  → PoseBus::update()  (fused pose for all consumers)
```

---

## Configuration Files (LittleFS /config/)

| File | Struct | Key fields |
|---|---|---|
| `wifi.cfg` | `WifiRuntimeConfig` | SSID, password, hostname, mDNS |
| `motion.cfg` | `MotionRuntimeConfig` | speed limits, heading PID gains, slew rate |
| `system.cfg` | `SystemRuntimeConfig` | telemetry Hz, MIDI on/off, auto power |
| `arena.cfg` | `ArenaRuntimeConfig` | arena dimensions, tag size and corner offset |

All files use a simple `key=value` format parsed by `ConfigService`.

---

## WebSocket Binary Protocol Summary

All packets start with a `uint16_t` magic word followed by `uint32_t seq` and `uint32_t t_ms`.


| Direction | Magic | Packet |
|---|---|---|
| Robot → Host | (none) | Pose: `seq u32 \| t_ms u32 \| x_mm f32 \| y_mm f32 \| yaw_deg f32` |
| Robot → Host | `0x4154` | AprilTags: `count u8` + N × hit structs |
| Robot → Host | `0x5343` | 3D scan: `n u8` + N × `(x, y, z) f32` |
| Robot → Host | `0x4D41` | Occupancy map full |
| Robot → Host | `0x4D44` | Occupancy map delta |
| Robot → Host | `0x494D` | JPEG snapshot |
| Host → Robot | `0x4242` | Control: `seq u32 \| dir u8 \| speed f32` |
| Host → Robot | `0x5643` | Motion vector: `seq u32 \| long f32 \| lat f32 \| rot f32` |

Full packet layouts are documented in the `WebWS::send*()` method signatures.

---

## Hardware Pin Reference

See `lib/config/RobotConfig.h` for the canonical list. Summary:

| Pin | Function |
|---|---|
| 1 | USB detect (input) |
| 2 | Sleep toggle (input) |
| 3 | Battery ADC |
| 4 | WS2812 RGB LED data |
| 5 | I2C SDA |
| 6 | I2C SCL |
| 7 | Buzzer PWM |
| 8 | Servo 2 PWM |
| 9 | Servo 1 PWM |
| 43 | System enable (active high) |
| 44 | Interrupt input |
