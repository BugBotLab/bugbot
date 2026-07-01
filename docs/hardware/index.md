# Hardware Overview

BugBot is built around the Seeed Studio XIAO ESP32-S3 Sense on a custom carrier PCB (BugBotBoard alpha 3). A separate optical flow breakout board (BugBot OpticalFlow alpha 1) adds mouse-sensor-based motion tracking.

---

## The Robot

| | |
|---|---|
| **Main CPU** | Seeed Studio XIAO ESP32-S3 Sense |
| **Wireless** | ESP-NOW over 2.4 GHz Wi-Fi (802.11b/g/n) |
| **Drive** | 4× DC geared motors via TB6612FNG drivers |
| **IMU** | Bosch BNO085 — 9-axis (accel + gyro + magnetometer) |
| **Optical flow** | PixArt PMW3360 — motion tracking breakout |
| **Depth sensor** | ST VL53L5CX — 8 × 8 ToF array, via connector |
| **Camera** | OV5640 (QQVGA JPEG, 160 × 120) — on XIAO Sense |
| **Servo driver** | NXP PCA9685 — 16-channel I2C PWM |
| **LED** | WS2812 NeoPixel (1 RGB) |
| **Buzzer** | Piezo buzzer (on PCB) |
| **Power** | LiPo battery via JST ZH 2-pin, polyfuse + TVS protection |

---

## The Dongle

The dongle is a **Seeed Studio XIAO ESP32-C3** plugged into a USB port on the host PC. It bridges ESP-NOW packets to and from the host library over a 115 200 baud serial link.

One dongle can talk to multiple robots simultaneously — the host library multiplexes them by device ID.

---

## Component Details

### Seeed Studio XIAO ESP32-S3 Sense (Robot CPU)

- Xtensa LX7 dual-core @ up to 240 MHz
- 8 MB Flash, 8 MB PSRAM (used for camera frame buffers)
- 2.4 GHz Wi-Fi + Bluetooth 5
- OV5640 camera on the Sense expansion board

### Toshiba TB6612FNG (Motor Drivers)

Two TB6612FNG ICs (U2, U3) are fitted on the carrier PCB — one per motor pair. Each IC drives two DC motors independently using a PWM speed signal and two direction logic pins.

| | |
|---|---|
| **ICs** | 2 × TB6612FNG (U2, U3) |
| **Motors driven** | 4 total (2 per IC) |
| **PWM source** | ESP32-S3 GPIO |
| **Motor connectors** | 4 × JST ZH B2B-ZR (1.5 mm pitch, 2-pin) — J104–J107 |

### Bosch BNO085 (IMU)

| | |
|---|---|
| **Interface** | I2C (SDA = GPIO 5, SCL = GPIO 6) |
| **Axes** | 9-axis — accelerometer + gyroscope + magnetometer |
| **Use** | Heading, orientation, and odometry @ 200 Hz |

The BNO085 runs its own internal sensor fusion, outputting stable orientation data without requiring calibration in firmware.

### PixArt PMW3360 (Optical Flow)

The PMW3360 sits on a separate **BugBot OpticalFlow alpha 1** breakout board that connects to J1 on the carrier PCB.

| | |
|---|---|
| **Interface** | SPI |
| **Connector** | 7-pin JST GH 1.25 mm (J1 on main board) |
| **Power** | 1.8 V from onboard ME6211C18M5G LDO |
| **Use** | Ground-facing motion tracking (x/y displacement) |

### ST VL53L5CX (ToF)

| | |
|---|---|
| **Interface** | I2C (shared bus), via J102 (7-pin connector) |
| **Mode** | 8 × 8 zone time-of-flight ranging |
| **Max range** | ~400 cm (degrades in bright sunlight) |
| **Poll rate** | 30 Hz |
| **FoV** | 45° horizontal and vertical |

### OV5640 Camera

| | |
|---|---|
| **Resolution** | QQVGA — 160 × 120 px |
| **Format** | JPEG |
| **Frame buffers** | PSRAM (keeps DRAM free) |
| **Use** | AprilTag detection, MJPEG stream |

### NXP PCA9685 (PWM / Servo Driver)

| | |
|---|---|
| **I2C address** | `0x40` |
| **Interface** | I2C (shared bus) |
| **Channels** | 16 PWM outputs |
| **Use** | Servo control (up to 4 servos via J109–J112) |
| **Servo connectors** | 4 × 3-pin header, 2.54 mm pitch |

### WS2812 RGB LED

Single addressable NeoPixel. Controlled by `ActuatorService`. Set from Python with `bot.led(...)`.

### Piezo Buzzer

Onboard piezo buzzer (BZ501). Set from Python with `bot.beep(freq, ms)`.

### Power System

| | |
|---|---|
| **Battery connector** | JST ZH 1.5 mm pitch, 2-pin (J108) |
| **Protection** | Polyfuse (MF-MSMF300X-2) + TVS diode (SMB package) |
| **Filtering** | 47 µF × 2 + 100 µF bulk capacitors on battery rail |
| **Logic supply** | 3.3 V (via load switch and LDO) |
| **Motor supply** | Direct from battery via TB6612FNG |

---

## Connector Reference

All connectors are on the **BugBotBoard alpha 3** carrier PCB.

| Ref | Function | Type |
|---|---|---|
| J104 | Motor 1 | JST ZH B2B-ZR (2-pin) |
| J105 | Motor 2 | JST ZH B2B-ZR (2-pin) |
| J106 | Motor 3 | JST ZH B2B-ZR (2-pin) |
| J107 | Motor 4 | JST ZH B2B-ZR (2-pin) |
| J108 | Battery | JST ZH (2-pin) |
| J109–J112 | Servos 1–4 | 3-pin header, 2.54 mm |
| J102 | VL53L5CX ToF | 7-pin connector |
| J112 | IMU (external) | 7-pin connector |
| J1 | Optical flow (PMW3360) | 7-pin JST GH 1.25 mm |
| J101 | LED | 5-pin connector |
| J103 | Power switch | 2-pin connector |

---

## I2C Bus

All I2C devices share SDA = GPIO 5, SCL = GPIO 6 at 400 kHz. The firmware uses a FreeRTOS mutex to make bus access thread-safe.

| Device | Address |
|---|---|
| PCA9685 (servo driver) | `0x40` |
| BMI270 (primary IMU) | `0x68` |
| BNO055 (secondary IMU) | `0x29` |
| VL53L5CX (ToF) | auto-detected at init |

---

## Memory

| Region | Size | Use |
|---|---|---|
| Flash | 8 MB | Firmware + LittleFS (web UI, config) |
| PSRAM | 8 MB | Camera frame buffers |
| DRAM | ~320 KB usable | FreeRTOS tasks, stacks, heap (after Wi-Fi) |

---

## PCB Files

Both PCB designs were produced with KiCad. Manufacturing files (Gerbers, BOM, STEP) are in the Electronic design folder.

| Board | Folder |
|---|---|
| BugBotBoard alpha 3 | `Electronic design/BugBotBoard_alpha_3` |
| BugBot OpticalFlow alpha 1 | `Electronic design/BugBot_OpticalFlow_alpha_1` |
