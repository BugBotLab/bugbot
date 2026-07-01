// BugBot_Firmware.ino
// Main sketch for the BugBot autonomous robot (XIAO ESP32-S3 Sense).
//
// Responsibilities:
//   - Owns every service object and wires them together at startup
//   - Manages the I2C bus lifecycle (recovery, mutex, sleep/wake)
//   - Manages deep-sleep entry and wake (physical toggle switch)
//   - Runs the main loop: calibration tick, auto-power, USB battery LED, MIDI, WebSocket poll
//
// See ARCHITECTURE.md for full data-flow diagrams and threading model.
// See README.md for build requirements and first-time setup.

#include <Arduino.h>
#include <Wire.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <ESPmDNS.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <driver/gpio.h>
#include <math.h>
#include <soc/rtc_cntl_reg.h>

// "" Config """"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
#include "lib/config/RobotConfig.h"
#include "lib/config/AppConfig.h"

// "" Core data types """""""""""""""""""""""""""""""""""""""""""""""""""""""""""
#include "lib/core/OdometryLib.h"
#include "lib/core/PoseBus.h"
#include "lib/core/BusLocks.hpp"
#include "lib/core/DriveDefs.hpp"
#include "lib/core/Transforms.hpp"

// "" Drivers """""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
#include "lib/drivers/BoardPowerLib.h"

// "" Services """"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
#include "lib/services/OdomService.h"
#include "lib/services/MotionService.h"
#include "lib/services/LidarService.h"
#include "lib/services/ActuatorService.h"
#include "lib/services/ConfigService.h"
#include "lib/services/ConfigPortalService.h"
#include "lib/services/MidiService.h"

// "" Networking """"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
#include "lib/net/WebWS.h"
#include "lib/services/WiFiService.h"
#include "lib/services/EspNowService.h"
#include "lib/services/ScriptService.h"
#include "bugbot_shims.h"

// "" Vision + localisation """""""""""""""""""""""""""""""""""""""""""""""""""""
#include "lib/services/CameraAPI.h"
#include "lib/services/CameraService.h"
#include "lib/services/AprilTagService.h"
#include "lib/services/BlobService.h"
#include "lib/services/ContourService.h"
#include "lib/services/TinyMLService.h"
#include "lib/services/FaceDetectService.h"
#include "lib/core/KinematicModel.h"
#include "lib/services/CalibService.h"
#include "lib/services/DroidChime.h"

RTC_DATA_ATTR bool g_bootToSleepMode = false;

// """""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
// Global instances  -  static storage so they persist across setup()/loop().
// """""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
static RobotConfig     robotCfg = makeDefaultRobotConfig();
static RuntimeConfig   runtimeCfg = makeDefaultRuntimeConfig();

static PoseBus         pose;
static OdomService     odomSvc;
static MotionService   motionSvc;
static LidarService    lidarSvc;
static ActuatorService actuatorSvc;
static WebWS           webws;
static ConfigService   configSvc;
static ConfigPortalService configPortalSvc;
static MidiService     midiSvc;
static WiFiService     wifi;
static EspNowService   espnow;
static ScriptService   scriptSvc;
static BoardPowerLib   boardPower;
static CameraService   cameraSvc;
static AprilTagService aprilTagSvc;
static BlobService     blobSvc;
static ContourService  contourSvc;
static TinyMLService      tinyMLSvc;
static FaceDetectService  faceDetectSvc;
static KinematicModel  kinModel;
static CalibService    calibSvc;
static DroidChime      chime;

SemaphoreHandle_t g_i2cMutex = nullptr;

static bool g_cameraStarted       = false;
static bool g_networkStarted      = false;
static bool g_mdnsStarted         = false;
volatile bool g_sleepEntryInProgress = false;
static bool g_usbLedWasActive     = false;
static uint8_t g_savedLedR = 0, g_savedLedG = 0, g_savedLedB = 0;
static constexpr uint32_t kUsbLedWsGraceMs = 3000U;

static void stopI2C_() {
  if (g_i2cMutex && xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    Wire.end();
    xSemaphoreGive(g_i2cMutex);
    vSemaphoreDelete(g_i2cMutex);
    g_i2cMutex = nullptr;
    Serial.println("[BugBot] I2C shutdown complete");
  } else {
    Serial.println("[BugBot] I2C shutdown skipped (lock timeout)");
  }
}

static void startI2C_() {
  // I2C bus recovery: if the ESP32 reset mid-transaction the PCA9685 (or any
  // slave) may be holding SDA LOW waiting for clock pulses that never arrived.
  // Wire.begin() alone does not fix this; we must clock out the stuck byte first.
  {
    const int sda = robotCfg.pins.i2c_sda;
    const int scl = robotCfg.pins.i2c_scl;
    pinMode(scl, OUTPUT_OPEN_DRAIN);
    pinMode(sda, OUTPUT_OPEN_DRAIN);
    digitalWrite(scl, HIGH);
    digitalWrite(sda, HIGH);
    delayMicroseconds(10);
    for (int i = 0; i < 9; i++) {
      if (digitalRead(sda)) break;        // SDA released  -  bus is free
      digitalWrite(scl, LOW);  delayMicroseconds(5);
      digitalWrite(scl, HIGH); delayMicroseconds(5);
    }
    // STOP condition: SDA LOWHIGH while SCL HIGH
    digitalWrite(sda, LOW);  delayMicroseconds(5);
    digitalWrite(scl, HIGH); delayMicroseconds(5);
    digitalWrite(sda, HIGH); delayMicroseconds(5);
  }

  Wire.end();
  delay(2);
  Wire.begin(robotCfg.pins.i2c_sda, robotCfg.pins.i2c_scl);
  Wire.setClock(robotCfg.i2c_hz);
  Wire.setTimeOut(robotCfg.i2c_timeoutms);

  if (g_i2cMutex) {
    vSemaphoreDelete(g_i2cMutex);
    g_i2cMutex = nullptr;
  }
  g_i2cMutex = xSemaphoreCreateMutex();
  if (!g_i2cMutex) {
    Serial.println("[BugBot] WARNING: failed to recreate I2C mutex");
  } else {
    Serial.println("[BugBot] I2C mutex recreated");
  }
}

static void initSleepToggle_() {
  // Return pin to normal digital GPIO mode after any deep-sleep wake.
  rtc_gpio_deinit((gpio_num_t)robotCfg.pins.sleep_toggle);
  pinMode(robotCfg.pins.sleep_toggle, INPUT_PULLUP);
  delay(5);
}

static bool sleepToggleIsSleepPosition_() {
  return digitalRead(robotCfg.pins.sleep_toggle) == LOW;
}

static bool sleepToggleStableLow_() {
  static bool lastRaw = false;
  static uint32_t lastChangeMs = 0;

  const bool raw = sleepToggleIsSleepPosition_();
  const uint32_t now = millis();

  if (raw != lastRaw) {
    lastRaw = raw;
    lastChangeMs = now;
  }

  return raw && (now - lastChangeMs >= 80);
}

static void configureWakeFromSleep_() {
  const gpio_num_t wakeGpio = (gpio_num_t)robotCfg.pins.sleep_toggle;

  // ext0 wake uses the RTC IO domain. Because the switch is GND or floating,
  // we must hold the line HIGH in deep sleep with the RTC-domain pull-up.
  rtc_gpio_init(wakeGpio);
  rtc_gpio_set_direction(wakeGpio, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en(wakeGpio);
  rtc_gpio_pulldown_dis(wakeGpio);
  rtc_gpio_hold_dis(wakeGpio);

  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_ext0_wakeup(wakeGpio, 1);
}

static bool putPca9685IntoSleep_() {
  constexpr uint8_t PCA9685_ADDR = 0x40;
  constexpr uint8_t MODE1_REG    = 0x00;
  constexpr uint8_t PCA_MODE1_SLEEP_BIT = 0x10;

  if (!g_i2cMutex || !I2C_LOCK(pdMS_TO_TICKS(300))) {
    Serial.println("[BugBot] PCA9685 sleep lock timeout");
    return false;
  }

  bool ok = false;
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(MODE1_REG);
  if (Wire.endTransmission(false) == 0 && Wire.requestFrom((int)PCA9685_ADDR, 1) == 1) {
    uint8_t mode1 = Wire.read();
    Wire.beginTransmission(PCA9685_ADDR);
    Wire.write(MODE1_REG);
    Wire.write((uint8_t)(mode1 | PCA_MODE1_SLEEP_BIT));
    ok = (Wire.endTransmission() == 0);
  }

  I2C_UNLOCK();
  Serial.printf("[BugBot] PCA9685 sleep %s\n", ok ? "OK" : "FAILED");
  return ok;
}

static bool putBMI270IntoSleep_() {
  const bool ok = odomSvc.prepareForSleep();
  Serial.printf("[BugBot] BMI270 sleep %s\n", ok ? "OK" : "FAILED");
  return ok;
}

static void stopCameraForSleep_() {
  if (!g_cameraStarted) return;
  // CameraService::suspend() handles the full power-down sequence.
  // CameraAPI_end() also calls esp_camera_deinit() which is idempotent here.
  cameraSvc.suspend();
  CameraAPI_end();
  g_cameraStarted = false;
  Serial.println("[BugBot] camera stopped for sleep");
}

static void preparePinsForDeepSleep_() {
  // Safe explicit list only for this XIAO ESP32S3 Sense robot build.
  // Do not brute-force every GPIO on the module.
  const int pinsToFloat[] = {
    robotCfg.pins.i2c_sda,
    robotCfg.pins.i2c_scl,
    robotCfg.pins.rgb_led_data,
    robotCfg.pins.buzzer_pwm,
    robotCfg.pins.servo1_pwm,
    robotCfg.pins.servo2_pwm,
    robotCfg.pins.int_in,

    // XIAO ESP32S3 Sense camera-related pins used by CameraAPI.cpp.
    10, 11, 12, 13, 14, 15, 16, 17, 18,
    38, 39, 40, 47, 48
  };

  for (int pin : pinsToFloat) {
    if (pin < 0) continue;
    pinMode(pin, INPUT);
    gpio_pullup_dis((gpio_num_t)pin);
    gpio_pulldown_dis((gpio_num_t)pin);
  }
}


static bool waitForServicesIdle_(uint32_t timeoutMs) {
  const uint32_t t0 = millis();
  while ((millis() - t0) < timeoutMs) {
    const bool motionIdle   = motionSvc.isIdle();
    const bool lidarIdle    = lidarSvc.isIdle();
    const bool odomIdle     = odomSvc.isIdle();
    const bool aprilTagIdle = !g_cameraStarted || cameraSvc.isIdle();

    if (motionIdle && lidarIdle && odomIdle && aprilTagIdle) {
      return true;
    }
    delay(10);
  }
  return false;
}



static void requestSleepReboot_() {
  if (g_sleepEntryInProgress) return;
  g_sleepEntryInProgress = true;

  // Turn RGB off while the normal runtime LED driver is still initialized.
  actuatorSvc.ledOff();
  delay(20);

  g_bootToSleepMode = true;
  Serial.println("[BugBot] scheduling minimal sleep boot");
  delay(20);
  ESP.restart();
}

static void enterMinimalSleepBoot_() {
  Serial.println("[BugBot] minimal sleep-entry boot");

  // Confirm we are still actually being asked to sleep.
  if (!sleepToggleStableLow_()) {
    Serial.println("[BugBot] sleep toggle not stable LOW on minimal boot, aborting");
    g_sleepEntryInProgress = false;
    return;
  }

  // Minimal init only: I2C + safe outputs + peripheral low-power + deep sleep.
  startI2C_();

  motionSvc.setCommand(DriveDir::Stop, 0.0f);
  actuatorSvc.allOff();
  Serial.println("[BugBot] actuators off");

  // Camera intentionally NOT initialized in this boot mode.
  g_cameraStarted = false;

  // Best-effort peripheral sleep commands.
  putBMI270IntoSleep_();
  putPca9685IntoSleep_();
  preparePinsForDeepSleep_();

  configureWakeFromSleep_();

  boardPower.setSystemEnabled(false);
  Serial.printf("[BugBot] system power enabled=%d\n", boardPower.systemEnabled() ? 1 : 0);
  Serial.printf("[BugBot] deep sleep wake on GPIO%d HIGH\n", robotCfg.pins.sleep_toggle);
  Serial.println("[BugBot] Flip switch back to RUN to wake.");

  stopI2C_();
  delay(20);

  esp_deep_sleep_start();
}

static void enterDeepSleepFromToggle_() {
  if (g_sleepEntryInProgress) return;
  g_sleepEntryInProgress = true;

  Serial.println("[BugBot] >>> ENTER TOGGLE DEEP SLEEP <<<");

  if (!sleepToggleStableLow_()) {
    Serial.println("[BugBot] sleep toggle not stable LOW, aborting");
    g_sleepEntryInProgress = false;
    return;
  }

  motionSvc.setCommand(DriveDir::Stop, 0.0f);
  actuatorSvc.allOff();
  Serial.println("[BugBot] actuators off");
  delay(20);

  if (g_mdnsStarted) {
    MDNS.end();
    g_mdnsStarted = false;
    Serial.println("[BugBot] mdns stopped");
  }

  if (g_networkStarted) {
    webws.stop();
    wifi.stop();
    g_networkStarted = false;
    Serial.println("[BugBot] network stopped");
  }

  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();

  motionSvc.requestSleep();
  lidarSvc.requestSleep();
  odomSvc.requestSleep();
  if (g_cameraStarted) cameraSvc.requestSleep();
  Serial.println("[BugBot] sleep requested for motion/lidar/odom/camera");

  const bool allIdle = waitForServicesIdle_(1500);
  Serial.printf("[BugBot] service idle wait: %s\n", allIdle ? "OK" : "TIMEOUT");

  if (!allIdle) {
    Serial.printf("[BugBot] idle states motion=%d lidar=%d odom=%d\n",
                  motionSvc.isIdle() ? 1 : 0,
                  lidarSvc.isIdle() ? 1 : 0,
                  odomSvc.isIdle() ? 1 : 0);
    Serial.println("[BugBot] aborting sleep because not all services are idle");
    g_sleepEntryInProgress = false;
    return;
  }

  stopCameraForSleep_();
  putBMI270IntoSleep_();
  putPca9685IntoSleep_();
  preparePinsForDeepSleep_();

  configureWakeFromSleep_();

  boardPower.setSystemEnabled(false);
  Serial.printf("[BugBot] system power enabled=%d\n", boardPower.systemEnabled() ? 1 : 0);
  Serial.printf("[BugBot] deep sleep wake on GPIO%d HIGH\n", robotCfg.pins.sleep_toggle);
  Serial.println("[BugBot] Flip switch back to RUN to wake.");

  stopI2C_();
  delay(20);

  esp_deep_sleep_start();
}

static uint8_t lerpByte_(uint8_t a, uint8_t b, float t) {
  if (t <= 0.0f) return a;
  if (t >= 1.0f) return b;
  return (uint8_t)lroundf(a + (b - a) * t);
}

static void batteryColor_(float vBat, uint8_t& r, uint8_t& g, uint8_t& b) {
  constexpr float vLow  = 3.30f;
  constexpr float vMid  = 3.70f;
  constexpr float vHigh = 4.20f;

  b = 0;

  if (vBat <= vLow) {
    r = 255;
    g = 0;
    return;
  }

  if (vBat < vMid) {
    const float t = (vBat - vLow) / (vMid - vLow);
    r = 255;
    g = lerpByte_(0, 255, t);
    return;
  }

  if (vBat < vHigh) {
    const float t = (vBat - vMid) / (vHigh - vMid);
    r = lerpByte_(255, 0, t);
    g = 255;
    return;
  }

  r = 0;
  g = 255;
}

static bool usbPresentActiveHigh_() {
  return digitalRead(robotCfg.pins.usb_detect) == HIGH;
}

static bool allowUsbBatteryPulse_() {
  if (!usbPresentActiveHigh_()) return false;
  if (webws.connectedClients() > 0) return false;
  if (midiSvc.midiLedActive()) return false;
  const uint32_t now = millis();
  const uint32_t sinceDisconnect = now - webws.lastDisconnectMs();
  return sinceDisconnect >= kUsbLedWsGraceMs;
}

static void updateUsbBatteryLed_() {
  if (actuatorSvc.ledLocked()) return;   // DroidChime owns the LED right now
  const bool pulseAllowed = allowUsbBatteryPulse_();

  if (!pulseAllowed) {
    if (g_usbLedWasActive) {
      actuatorSvc.setWsLedWritesEnabled(true);
      if (!midiSvc.midiLedActive()) {
        actuatorSvc.setLed(g_savedLedR, g_savedLedG, g_savedLedB);
      }
      g_usbLedWasActive = false;
    }
    return;
  }

  if (!g_usbLedWasActive) {
    g_savedLedR = actuatorSvc.ledR();
    g_savedLedG = actuatorSvc.ledG();
    g_savedLedB = actuatorSvc.ledB();
    actuatorSvc.setWsLedWritesEnabled(false);
    g_usbLedWasActive = true;
  }

  static uint32_t lastUpdateMs = 0;
  const uint32_t now = millis();
  if (now - lastUpdateMs < 20U) return;
  lastUpdateMs = now;

  uint8_t baseR = 0, baseG = 0, baseB = 0;
  batteryColor_(boardPower.readBatteryVolts(), baseR, baseG, baseB);

  const uint32_t periodMs = 2500U;
  const uint32_t ms = now % periodMs;
  const float phase = (float)ms / (float)periodMs;
  float brightness = 0.5f - 0.5f * cosf(2.0f * PI * phase);
  brightness = 0.35f + 0.65f * brightness;

  actuatorSvc.setLed(
    (uint8_t)lroundf(baseR * brightness),
    (uint8_t)lroundf(baseG * brightness),
    (uint8_t)lroundf(baseB * brightness)
  );
}

static void printRuntimeConfig_() {
  Serial.println("[CONFIG] ===== Active runtime config =====");
  Serial.printf("[CONFIG] wifi sta=%d ssid='%s' ap=%d ap_ssid='%s' host=%s mdns=%d timeout=%lu\n",
                runtimeCfg.wifi.staEnabled ? 1 : 0,
                runtimeCfg.wifi.staSsid.c_str(),
                runtimeCfg.wifi.apEnabled ? 1 : 0,
                runtimeCfg.wifi.apSsid.c_str(),
                runtimeCfg.wifi.hostname.c_str(),
                runtimeCfg.wifi.mdnsEnabled ? 1 : 0,
                (unsigned long)runtimeCfg.wifi.connectTimeoutMs);
  Serial.printf("[CONFIG] motion field=%d lin=%.2f rot=%.2f ctrlHz=%lu hold=%d slew=%d\n",
                runtimeCfg.motion.fieldCentric ? 1 : 0,
                runtimeCfg.motion.maxLinearSpeed,
                runtimeCfg.motion.maxRotSpeed,
                (unsigned long)runtimeCfg.motion.controlRateHz,
                runtimeCfg.motion.headingHoldEnabled ? 1 : 0,
                runtimeCfg.motion.slewLimitEnabled ? 1 : 0);
  Serial.printf("[CONFIG] system bootDelay=%lu logToSd=%d telemetry=%lu ui=%lu\n",
                (unsigned long)runtimeCfg.system.bootDelayMs,
                runtimeCfg.system.logToSd ? 1 : 0,
                (unsigned long)runtimeCfg.system.telemetryRateHz,
                (unsigned long)runtimeCfg.system.uiUpdateRateHz);
}

static void loadRuntimeConfig_() {
  runtimeCfg = makeDefaultRuntimeConfig();
  const bool loaded = configSvc.beginAndLoad(runtimeCfg);
  if (!loaded && !runtimeCfg.system.safeBootOnConfigError) {
    Serial.println("[CONFIG] Config load failed and safe-boot disabled");
  }
  motionSvc.configure(runtimeCfg.motion);
  printRuntimeConfig_();
}

static void updateUsbState_() {

  static bool first = true;
  static bool lastUsb = false;

  const bool usb = usbPresentActiveHigh_();

  if (first) {
    Serial.printf("[BugBot] usb init: %d\n", usb ? 1 : 0);
    lastUsb = usb;
    first = false;
    return;
  }

  if (usb != lastUsb) {
    Serial.printf("[BugBot] usb transition: %d -> %d\n", lastUsb ? 1 : 0, usb ? 1 : 0);
    lastUsb = usb;
  }
}


// """""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
// setup()  -  runs once on boot.
//   1. Minimal sleep path: if RTC flag is set, skip full init and go straight to deep sleep.
//   2. Normal path: I2C, config, power rail, services, network, vision, pose, diagnostics.
// """""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
void setup() {
  Serial.begin(115200);
  delay(200);

  // "" Minimal-sleep boot (restarted by requestSleepReboot_ to avoid mid-task shutdown) ""
  if (g_bootToSleepMode) {
    Serial.println("\n[BugBot] minimal-sleep boot");
    g_bootToSleepMode = false;
    enterMinimalSleepBoot_();
    return;
  }

  Serial.println("\n[BugBot] setup()");

  initSleepToggle_();

  const esp_sleep_wakeup_cause_t wakeCause = esp_sleep_get_wakeup_cause();
  Serial.printf("[BugBot] wake cause=%d, sleepSwitch=%d (%s)\n",
                (int)wakeCause,
                digitalRead(robotCfg.pins.sleep_toggle),
                sleepToggleIsSleepPosition_() ? "SLEEP" : "RUN");

  // "" System init """"""""""""""""""""""""""""""""""""""""""""""""""""""""""""
  setCpuFrequencyMhz(240);
  startI2C_();
  loadRuntimeConfig_();
  if (runtimeCfg.system.bootDelayMs > 0) {
    delay(runtimeCfg.system.bootDelayMs);
  }

  BoardPowerLib::Config pwrCfg;
  pwrCfg.pinEnable    = robotCfg.pins.sys_enable;
  pwrCfg.pinUsbDetect = robotCfg.pins.usb_detect;
  pwrCfg.pinBatAdc    = robotCfg.pins.bat_adc;
  boardPower.configure(pwrCfg);
  boardPower.begin();

  if (sleepToggleIsSleepPosition_()) {
    Serial.println("[BugBot] Switch is in SLEEP position at boot");
    configureWakeFromSleep_();
    Serial.println("[BugBot] Entering deep sleep immediately");
    boardPower.setSystemEnabled(false);
    delay(20);
    esp_deep_sleep_start();
  }

  // "" Actuators + boot chime """""""""""""""""""""""""""""""""""""""""""""""""
  actuatorSvc.begin(robotCfg, &boardPower, runtimeCfg.system.autoPeripheralPower, runtimeCfg.system.peripheralIdleTimeoutMs, runtimeCfg.system.peripheralPowerOnSettleMs);
  actuatorSvc.setWsLedWritesEnabled(true);
  chime.begin(actuatorSvc, /*prio=*/1, /*core=*/1);
  updateUsbState_();
  updateUsbBatteryLed_();

  Serial.printf("[BugBot] USB present(raw): %s\n", usbPresentActiveHigh_() ? "YES" : "NO");
  Serial.printf("[BugBot] Battery estimate: %.2f V\n", boardPower.readBatteryVolts());

  if (sleepToggleStableLow_()) {
    Serial.println("[BugBot] Sleep switch moved to SLEEP");
    requestSleepReboot_();
  }

  // Normal running stays enabled whether USB is plugged or not.
  boardPower.setSystemEnabled(true);
  Serial.printf("[BugBot] system power enabled=%d\n",
                boardPower.systemEnabled() ? 1 : 0);

  // I2C device scan — printed once at boot to confirm PCA9685 (0x40) is alive.
  if (I2C_LOCK(pdMS_TO_TICKS(500))) {
    Serial.print("[I2C scan] devices at:");
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.printf(" 0x%02X", addr);
      }
    }
    Serial.println();
    I2C_UNLOCK();
  }

  // "" Motion + sensors """"""""""""""""""""""""""""""""""""""""""""""""""""""
  odomSvc.start(pose, robotCfg.odom_hz);
  motionSvc.configureAutoPower(runtimeCfg.system.autoPeripheralPower, runtimeCfg.system.peripheralIdleTimeoutMs, runtimeCfg.system.peripheralPowerOnSettleMs);
  motionSvc.start(pose, &boardPower);
  lidarSvc.start(robotCfg.lidar_row, robotCfg.lidar_hz);

  // "" Comms mode selection """""""""""""""""""""""""""""""""""""""""""""""""""
  // ESP-NOW mode is the SOLE transport when enabled: every WiFi-based feature
  // (STA/AP, WebSocket, HTTP config portal, mDNS, MIDI, UDP telemetry) and the
  // WebSocket-driven vision/pose stack are skipped. Default (espnowEnabled=0)
  // takes the existing WiFi path below, byte-for-byte unchanged.
  if (runtimeCfg.system.espnowEnabled) {
    Serial.println("[BugBot] ESP-NOW mode: WiFi/AP/WS/HTTP/mDNS/MIDI/UDP all DISABLED");
    espnow.attach(&motionSvc, &actuatorSvc, &lidarSvc, &pose, &boardPower);
    bugbot_shims_init(&motionSvc, &actuatorSvc, &lidarSvc, &pose, &aprilTagSvc, &boardPower, &odomSvc);
    bugbot_shims_set_camera(&cameraSvc);
    scriptSvc.attach(&espnow);
    espnow.attachScript(&scriptSvc);
    if (!espnow.begin(runtimeCfg.system)) {
      Serial.println("[BugBot] ESP-NOW bring-up FAILED");
    }
    // Start camera and CV services (WebWS has no clients in ESP-NOW mode, so
    // sendAprilTags() is a no-op, but detection still updates the shim cache).
    aprilTagSvc.begin(webws, 37.5f, 66.0f);
    blobSvc.begin();
    contourSvc.begin();
    tinyMLSvc.begin();
    tinyMLSvc.loadModel();
    faceDetectSvc.begin();
    bugbot_shims_set_cv_services(&blobSvc, &contourSvc, &tinyMLSvc);
    bugbot_shims_set_face_service(&faceDetectSvc);
    g_cameraStarted = cameraSvc.begin();
    if (!g_cameraStarted) {
      Serial.println("[BugBot] Camera service failed to start (ESP-NOW mode)");
    }
    if (g_cameraStarted) {
      cameraSvc.setConsumer(&aprilTagSvc);
    }
    aprilTagSvc.setTagCallback([](uint8_t n, const WebWS::AprilTagHit* hits) {
      bugbot_shim_on_apriltags(n, (const BugBotTagHit*)hits);
      for (uint8_t i = 0; i < n; i++) chime.notifyTagId(hits[i].id);
    });

    chime.play(DroidChime::STARTUP);   // "systems ready" boot chime
    return;
  }

  // "" Network + WebSocket """"""""""""""""""""""""""""""""""""""""""""""""""""
  webws.serveFilesFromFS(true);
  configPortalSvc.attach(webws, configSvc, runtimeCfg);

  wifi.configure(
    runtimeCfg.wifi.staEnabled,
    runtimeCfg.wifi.apEnabled,
    runtimeCfg.wifi.staSsid.c_str(),
    runtimeCfg.wifi.staPassword.c_str(),
    runtimeCfg.wifi.wifiSleep,
    runtimeCfg.wifi.apSsid.c_str(),
    runtimeCfg.wifi.apPassword.c_str(),
    runtimeCfg.wifi.connectTimeoutMs
  );
  wifi.setHostname(runtimeCfg.wifi.hostname.c_str());

  webws.onControl([](uint8_t dir, float speed) {
    if (g_sleepEntryInProgress) return;
    motionSvc.setCommand(static_cast<DriveDir>(dir), speed);
  });

  webws.onMotionVec([](float longitudinal, float lateral, float rotational) {
    if (g_sleepEntryInProgress || calibSvc.isRunning()) return;
    motionSvc.setCommandVec(longitudinal, lateral, rotational);
  });

  const bool net_ok = runtimeCfg.system.udpMessagesEnabled
    ? wifi.startWithTelemetry(
    webws,
    pose,
    lidarSvc,
    motionSvc,
    boardPower,
    runtimeCfg.system.telemetryRateHz,
    runtimeCfg.system.telemetryRateHz
  )
    : wifi.start(webws);

  g_networkStarted = net_ok;

  if (!net_ok) {
    Serial.println("[BugBot] Network bring-up FAILED");
  } else {
    if (runtimeCfg.system.midiEnabled) {
      midiSvc.begin(&actuatorSvc, &motionSvc);
    } else {
      Serial.println("[MIDI] disabled by config");
    }
    if (runtimeCfg.wifi.mdnsEnabled) {
      if (MDNS.begin(runtimeCfg.wifi.hostname.c_str())) {
        Serial.printf("[BugBot] mDNS ready: http://%s.local/config\n", runtimeCfg.wifi.hostname.c_str());
        g_mdnsStarted = true;
      } else {
        Serial.println("[BugBot] mDNS start failed");
      }
    }
  }

  webws.onClientConnect([]() { chime.play(DroidChime::CONNECTED); });
  webws.onClientDisconnect([]() { chime.play(DroidChime::DISCONNECTED); });

  webws.onText([](const char* msg) {
    if (g_sleepEntryInProgress) return;
    if (strstr(msg, "\"cmd\"")) {
      if (strstr(msg, "\"calibrate\"")) {
        if (strstr(msg, "cancel")) calibSvc.cancel();
        else { calibSvc.trigger(); chime.play(DroidChime::CALIB_START); }
      }
      return;
    }
    if (strstr(msg, "\"camera\"")) {
      if (strstr(msg, "\"on\""))  cameraSvc.resume();
      if (strstr(msg, "\"off\"")) cameraSvc.suspend();
      if (strstr(msg, "\"cv\"")) {
        if (strstr(msg, "\"apriltag\"")) cameraSvc.setConsumer(&aprilTagSvc);
        else if (strstr(msg, "\"blob\""))    cameraSvc.setConsumer(&blobSvc);
        else if (strstr(msg, "\"contour\"")) cameraSvc.setConsumer(&contourSvc);
        else if (strstr(msg, "\"tinyml\""))  cameraSvc.setConsumer(&tinyMLSvc);
        else if (strstr(msg, "\"face\""))     cameraSvc.setConsumer(&faceDetectSvc);
        else if (strstr(msg, "\"none\""))    cameraSvc.setConsumer(nullptr);
      }
      return;
    }
    // Runtime field-centric / heading-hold toggle
    auto parseBoolJson_ = [](const char* s, const char* key, bool& out) -> bool {
      const char* p = strstr(s, key); if (!p) return false;
      p = strchr(p, ':');             if (!p) return false;
      while (*++p == ' ') {}
      if (strncmp(p, "true",  4) == 0) { out = true;  return true; }
      if (strncmp(p, "false", 5) == 0) { out = false; return true; }
      return false;
    };
    bool boolVal;
    if (parseBoolJson_(msg, "\"field_centric\"", boolVal)) {
      runtimeCfg.motion.fieldCentric = boolVal;
      motionSvc.configure(runtimeCfg.motion);
    }
    if (parseBoolJson_(msg, "\"heading_hold_enabled\"", boolVal)) {
      runtimeCfg.motion.headingHoldEnabled = boolVal;
      motionSvc.configure(runtimeCfg.motion);
    }

    actuatorSvc.handleWsText(msg);
  });

  // "" Vision + pose estimation """""""""""""""""""""""""""""""""""""""""""""""
  // CV services: begin() just stores parameters; onActivate() allocates memory.
  aprilTagSvc.begin(webws, 37.5f, 66.0f);
  blobSvc.begin();
  contourSvc.begin();
  tinyMLSvc.begin();
  tinyMLSvc.loadModel();
  faceDetectSvc.begin();
  bugbot_shims_set_cv_services(&blobSvc, &contourSvc, &tinyMLSvc);
  bugbot_shims_set_face_service(&faceDetectSvc);

  // Start camera task; default to AprilTag as the active consumer.
  g_cameraStarted = cameraSvc.begin();
  if (!g_cameraStarted) {
    Serial.println("[BugBot] Camera service failed to start");
  }
  // MJPEG stream on :81  shares camera frames with the CV consumer.
  CameraAPI_begin(81);
  if (g_cameraStarted) {
    cameraSvc.setConsumer(&aprilTagSvc);
    bugbot_shims_set_camera(&cameraSvc);
    cameraSvc.suspend();
    Serial.println("[BugBot] camera suspended on startup (thermal test)");
  }

  // Kinematic calibration
  calibSvc.begin(motionSvc, pose, webws);
  calibSvc.loadFromFlash();
  aprilTagSvc.setTagCallback([](uint8_t n, const WebWS::AprilTagHit* hits) {
    calibSvc.onAprilTags(n, hits);
    for (uint8_t i = 0; i < n; i++) chime.notifyTagId(hits[i].id);
    bugbot_shim_on_apriltags(n, (const BugBotTagHit*)hits);
  });

  if (g_networkStarted) {
    Serial.printf("[BugBot] IP=%s  UI: http://%s/config\n",
                  wifi.ip().toString().c_str(),
                  wifi.ip().toString().c_str());
    if (g_mdnsStarted) {
      Serial.printf("[BugBot] mDNS UI: http://%s.local/config\n", runtimeCfg.wifi.hostname.c_str());
    }
  }

  chime.play(DroidChime::STARTUP);   // "systems ready" boot chime
}

// """""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
// Serial config shell  -  type key=value to change a setting live.
// """""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

static void printAllConfig_() {
  const auto& c = runtimeCfg;
  const auto b = [](bool v) -> const char* { return v ? "1" : "0"; };
  Serial.println("[CFG] === wifi ===");
  Serial.printf("[CFG] sta_enabled=%s\n",             b(c.wifi.staEnabled));
  Serial.printf("[CFG] sta_ssid=%s\n",                c.wifi.staSsid.c_str());
  Serial.printf("[CFG] sta_password=%s\n",            c.wifi.staPassword.c_str());
  Serial.printf("[CFG] ap_enabled=%s\n",              b(c.wifi.apEnabled));
  Serial.printf("[CFG] ap_ssid=%s\n",                 c.wifi.apSsid.c_str());
  Serial.printf("[CFG] ap_password=%s\n",             c.wifi.apPassword.c_str());
  Serial.printf("[CFG] hostname=%s\n",                c.wifi.hostname.c_str());
  Serial.printf("[CFG] mdns_enabled=%s\n",            b(c.wifi.mdnsEnabled));
  Serial.printf("[CFG] connect_timeout_ms=%lu\n",     (unsigned long)c.wifi.connectTimeoutMs);
  Serial.printf("[CFG] wifi_sleep=%s\n",              b(c.wifi.wifiSleep));
  Serial.println("[CFG] === motion ===");
  Serial.printf("[CFG] field_centric=%s\n",           b(c.motion.fieldCentric));
  Serial.printf("[CFG] max_linear_speed=%.3f\n",      c.motion.maxLinearSpeed);
  Serial.printf("[CFG] max_rot_speed=%.3f\n",         c.motion.maxRotSpeed);
  Serial.printf("[CFG] input_deadband=%.3f\n",        c.motion.inputDeadband);
  Serial.printf("[CFG] control_rate_hz=%lu\n",        (unsigned long)c.motion.controlRateHz);
  Serial.printf("[CFG] teleop_timeout_ms=%lu\n",      (unsigned long)c.motion.teleopTimeoutMs);
  Serial.printf("[CFG] heading_hold_enabled=%s\n",    b(c.motion.headingHoldEnabled));
  Serial.printf("[CFG] heading_kp=%.6f\n",            c.motion.headingKp);
  Serial.printf("[CFG] heading_ki=%.6f\n",            c.motion.headingKi);
  Serial.printf("[CFG] heading_kd=%.6f\n",            c.motion.headingKd);
  Serial.printf("[CFG] slew_limit_enabled=%s\n",      b(c.motion.slewLimitEnabled));
  Serial.printf("[CFG] slew_rate=%.3f\n",             c.motion.slewRate);
  Serial.println("[CFG] === system ===");
  Serial.printf("[CFG] boot_delay_ms=%lu\n",          (unsigned long)c.system.bootDelayMs);
  Serial.printf("[CFG] serial_baud=%lu\n",            (unsigned long)c.system.serialBaud);
  Serial.printf("[CFG] safe_boot_on_config_error=%s\n", b(c.system.safeBootOnConfigError));
  Serial.printf("[CFG] log_to_sd=%s\n",               b(c.system.logToSd));
  Serial.printf("[CFG] telemetry_rate_hz=%lu\n",      (unsigned long)c.system.telemetryRateHz);
  Serial.printf("[CFG] ui_update_rate_hz=%lu\n",      (unsigned long)c.system.uiUpdateRateHz);
  Serial.printf("[CFG] start_web_ui_on_boot=%s\n",    b(c.system.startWebUiOnBoot));
  Serial.printf("[CFG] midi_enabled=%s\n",            b(c.system.midiEnabled));
  Serial.printf("[CFG] udp_messages_enabled=%s\n",    b(c.system.udpMessagesEnabled));
  Serial.printf("[CFG] auto_peripheral_power=%s\n",   b(c.system.autoPeripheralPower));
  Serial.printf("[CFG] peripheral_idle_timeout_ms=%lu\n",  (unsigned long)c.system.peripheralIdleTimeoutMs);
  Serial.printf("[CFG] peripheral_power_on_settle_ms=%lu\n", (unsigned long)c.system.peripheralPowerOnSettleMs);
  Serial.println("[CFG] === espnow ===");
  Serial.printf("[CFG] espnow_enabled=%s\n",          b(c.system.espnowEnabled));
  Serial.printf("[CFG] espnow_channel=%u\n",          c.system.espnowChannel);
  Serial.printf("[CFG] device_id=%s\n",               c.system.deviceId.c_str());
  Serial.printf("[CFG] pairing_key=%s\n",             c.system.pairingKey.isEmpty() ? "(unset)" : "(set, hidden)");
  Serial.println("[CFG] === arena ===");
  Serial.printf("[CFG] width_mm=%.1f\n",              c.arena.widthMm);
  Serial.printf("[CFG] height_mm=%.1f\n",             c.arena.heightMm);
  Serial.printf("[CFG] tag_center_from_corner_mm=%.1f\n", c.arena.tagCenterFromCornerMm);
  Serial.printf("[CFG] tag_size_mm=%.1f\n",           c.arena.tagSizeMm);
}

static void printConfigHelp_() {
  Serial.println("[CFG] Commands:");
  Serial.println("[CFG]   list               -  print all current values");
  Serial.println("[CFG]   save               -  write current config to flash");
  Serial.println("[CFG]   reload             -  reload config from flash");
  Serial.println("[CFG]   help               -  this message");
  Serial.println("[CFG]   key=value          -  set a config key and save");
  Serial.println("[CFG]   espnow_init        -  provision id+key, enable ESP-NOW, print ESPNOW_INIT, reboot");
  Serial.println("[CFG]   regenerate_key     -  roll pairing_key only (re-bind host), print ESPNOW_INIT");
  Serial.println("[CFG]   espnow_off         -  disable ESP-NOW, reboot into WiFi mode");
  Serial.println("[CFG] Keys (wifi): sta_enabled sta_ssid sta_password ap_enabled ap_ssid");
  Serial.println("[CFG]              ap_password hostname mdns_enabled connect_timeout_ms wifi_sleep");
  Serial.println("[CFG] Keys (motion): field_centric max_linear_speed max_rot_speed input_deadband");
  Serial.println("[CFG]               control_rate_hz teleop_timeout_ms heading_hold_enabled");
  Serial.println("[CFG]               heading_kp heading_ki heading_kd slew_limit_enabled slew_rate");
  Serial.println("[CFG] Keys (system): boot_delay_ms serial_baud safe_boot_on_config_error log_to_sd");
  Serial.println("[CFG]               telemetry_rate_hz ui_update_rate_hz start_web_ui_on_boot");
  Serial.println("[CFG]               midi_enabled udp_messages_enabled auto_peripheral_power");
  Serial.println("[CFG]               peripheral_idle_timeout_ms peripheral_power_on_settle_ms");
  Serial.println("[CFG] Keys (espnow): espnow_enabled espnow_channel device_id");
  Serial.println("[CFG]               (pairing_key is set only via espnow_init / regenerate_key)");
  Serial.println("[CFG] Keys (arena): width_mm height_mm tag_center_from_corner_mm tag_size_mm");
  Serial.println("[CFG] Note: wifi/system/arena changes take effect on next reboot.");
  Serial.println("[CFG]       motion changes apply immediately.");
}

// Generate 16 hex chars (64-bit) from two esp_random() draws.
static String randomHex64_() {
  uint32_t hi = esp_random();
  uint32_t lo = esp_random();
  char buf[17];
  snprintf(buf, sizeof(buf), "%08x%08x", (unsigned)hi, (unsigned)lo);
  return String(buf);
}

// Print the machine-readable provisioning line the host init script consumes.
//   ESPNOW_INIT id=<16hex> key=<16hex> mac=<aabbccddeeff> ch=<n> fw=<ver>
static void printEspnowInitLine_() {
  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char macStr[13];
  snprintf(macStr, sizeof(macStr), "%02x%02x%02x%02x%02x%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.printf("ESPNOW_INIT id=%s key=%s mac=%s ch=%u fw=%u\n",
                runtimeCfg.system.deviceId.c_str(),
                runtimeCfg.system.pairingKey.c_str(),
                macStr,
                runtimeCfg.system.espnowChannel,
                (unsigned)ESPNOW_FW_VERSION);
}

static void handleSerialLine_(const String& line) {
  // "" ESP-NOW provisioning commands """"""""""""""""""""""""""""""""""""""""""
  if (line == "espnow_init") {
    // 1. Stable device_id: generate once, keep across re-inits.
    if (runtimeCfg.system.deviceId.isEmpty()) {
      runtimeCfg.system.deviceId = randomHex64_();
    }
    // 2. ALWAYS roll the pairing_key (this is what locks out old hosts).
    runtimeCfg.system.pairingKey = randomHex64_();
    // 3. Enable ESP-NOW mode.
    runtimeCfg.system.espnowEnabled = true;
    if (!configSvc.saveAll(runtimeCfg)) {
      Serial.println("[CFG] espnow_init: save FAILED");
      return;
    }
    // 4. Machine-readable line for the host init script.
    printEspnowInitLine_();
    // 5. Reboot so the comms mode takes effect.
    Serial.println("[CFG] espnow_init OK  -  rebooting into ESP-NOW mode");
    delay(50);
    ESP.restart();
    return;
  }
  if (line == "regenerate_key") {
    // Roll ONLY the pairing_key (re-bind to a new host, same identity).
    if (runtimeCfg.system.deviceId.isEmpty()) {
      runtimeCfg.system.deviceId = randomHex64_();
    }
    runtimeCfg.system.pairingKey = randomHex64_();
    if (!configSvc.saveAll(runtimeCfg)) {
      Serial.println("[CFG] regenerate_key: save FAILED");
      return;
    }
    printEspnowInitLine_();
    Serial.println("[CFG] regenerate_key OK (no reboot)");
    return;
  }
  if (line == "espnow_off") {
    runtimeCfg.system.espnowEnabled = false;
    if (!configSvc.saveAll(runtimeCfg)) {
      Serial.println("[CFG] espnow_off: save FAILED");
      return;
    }
    Serial.println("[CFG] espnow_off OK  -  rebooting into WiFi mode");
    delay(50);
    ESP.restart();
    return;
  }

  if (line == "flash") {
    Serial.println("[BugBot] entering download mode...");
    Serial.flush();
    delay(100);
    REG_SET_BIT(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
    esp_restart();
    return;
  }

  if (line == "camera_on") {
    cameraSvc.resume();
    Serial.println("[BugBot] camera resume requested");
    return;
  }
  if (line == "camera_off") {
    cameraSvc.suspend();
    Serial.println("[BugBot] camera suspend requested");
    return;
  }
  if (line == "temp") {
    Serial.printf("[TEMP] ESP32-S3 die: %.1f C\n", temperatureRead());
    return;
  }
  if (line == "list" || line == "config" || line == "cfg") {
    printAllConfig_();
    return;
  }
  if (line == "help" || line == "?") {
    printConfigHelp_();
    return;
  }
  if (line == "save") {
    if (configSvc.saveAll(runtimeCfg)) Serial.println("[CFG] Saved OK");
    else                               Serial.println("[CFG] Save FAILED");
    return;
  }
  if (line == "reload") {
    configSvc.reload(runtimeCfg);
    motionSvc.configure(runtimeCfg.motion);
    Serial.println("[CFG] Reloaded from flash");
    return;
  }
  const int eq = line.indexOf('=');
  if (eq < 1) {
    Serial.printf("[CFG] Unknown command: %s  (type 'help')\n", line.c_str());
    return;
  }
  String key = line.substring(0, eq);
  String val = line.substring(eq + 1);
  key.trim(); val.trim();
  if (configSvc.setKey(key, val, runtimeCfg)) {
    motionSvc.configure(runtimeCfg.motion);
    if (configSvc.saveAll(runtimeCfg)) {
      Serial.printf("[CFG] OK: %s = %s  (saved)\n", key.c_str(), val.c_str());
    } else {
      Serial.printf("[CFG] Set OK but save FAILED: %s\n", key.c_str());
    }
  } else {
    Serial.printf("[CFG] Unknown key: %s  (type 'help')\n", key.c_str());
  }
}

static void pollSerialConfig_() {
  static String buf;
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf.trim();
      if (buf.length() > 0) handleSerialLine_(buf);
      buf = "";
    } else {
      if (buf.length() < 256) buf += c;
    }
  }
}

// """""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
// loop()  -  runs continuously on Core 1.
//   Handles non-task work: calibration state machine, power rail management,
//   sleep-toggle detection, battery LED, low-battery chime, gyro auto-recal,
//   MIDI polling, and WebSocket housekeeping.
// """""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
void loop() {
  // "" ESP-NOW mode loop """"""""""""""""""""""""""""""""""""""""""""""""""""""
  // Self-contained: serial config shell, ESP-NOW pump (BEACON + deadman + lease),
  // sleep toggle, battery LED, low-battery chime. None of the WiFi/WebSocket/MIDI
  // housekeeping below runs in this mode.
  if (runtimeCfg.system.espnowEnabled) {
    pollSerialConfig_();
    espnow.tick();

    // Keep the system rail enabled during normal running (USB must not kill the robot).
    if (!g_sleepEntryInProgress && !boardPower.systemEnabled()) {
      boardPower.setSystemEnabled(true);
      motionSvc.signalPowerRestored();
    }

    updateUsbState_();
    updateUsbBatteryLed_();

    if (sleepToggleStableLow_()) {
      Serial.println("[BugBot] Sleep switch moved to SLEEP");
      requestSleepReboot_();
    }

    {
      static uint32_t lastLowBatChimeMs = 0;
      const uint32_t nowMs = millis();
      if (nowMs - lastLowBatChimeMs >= 60000) {
        const float bat = boardPower.readBatteryVolts();
        if (bat > 0.5f && bat < 3.5f) {
          chime.play(DroidChime::LOW_BATTERY);
          lastLowBatChimeMs = nowMs;
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(2));
    return;
  }

  pollSerialConfig_();
  calibSvc.tick();

  // USB should never disable the live robot. Keep the external system rail enabled during normal running.
  if (!g_sleepEntryInProgress && !runtimeCfg.system.autoPeripheralPower && !boardPower.systemEnabled()) {
    boardPower.setSystemEnabled(true);
    motionSvc.signalPowerRestored();
    Serial.println("[BugBot] Re-enabled system power in loop");
  }

  updateUsbState_();
  updateUsbBatteryLed_();

  if (runtimeCfg.system.autoPeripheralPower && !g_sleepEntryInProgress) {
    const uint32_t nowMs = millis();
    const bool wantPeripheralPower =
      motionSvc.wantsPeripheralPower(nowMs) ||
      actuatorSvc.wantsPeripheralPower(nowMs) ||
      webws.connectedClients() > 0;

    if (wantPeripheralPower) {
      if (!boardPower.systemEnabled()) {
        boardPower.setSystemEnabled(true);
        motionSvc.signalPowerRestored();
      }
    } else {
      if (boardPower.systemEnabled()) {
        boardPower.setSystemEnabled(false);
      }
    }
  }

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 20000) {
    lastPrint = millis();
    const int usbRaw = digitalRead(robotCfg.pins.usb_detect);
    const int sleepRaw = digitalRead(robotCfg.pins.sleep_toggle);
    Serial.printf("[PWR] usbRaw=%d usb=%d sleepRaw=%d enabled=%d ws=%u bat=%.2f\n",
                  usbRaw,
                  usbPresentActiveHigh_() ? 1 : 0,
                  sleepRaw,
                  boardPower.systemEnabled() ? 1 : 0,
                  (unsigned)webws.connectedClients(),
                  boardPower.readBatteryVolts());
  }

  if (sleepToggleStableLow_()) {
    Serial.println("[BugBot] Sleep switch moved to SLEEP");
    requestSleepReboot_();
  }

  // Low-battery audio warning: plays at most once per 60 s to avoid constant beeping.
  {
    static uint32_t lastLowBatChimeMs = 0;
    const uint32_t nowMs = millis();
    if (nowMs - lastLowBatChimeMs >= 60000) {
      const float bat = boardPower.readBatteryVolts();
      if (bat > 0.5f && bat < 3.5f) {
        chime.play(DroidChime::LOW_BATTERY);
        lastLowBatChimeMs = nowMs;
      }
    }
  }

  // Auto-recalibrate gyro bias when robot has been stopped for 3 s.
  // Corrects vibration-induced bias shift that accumulates during motor use.
  {
    static uint32_t lastMoveMs = 0;
    static bool     biasCalDone = false;
    if (motionSvc.isIdle()) {
      const uint32_t nowMs = millis();
      if (!biasCalDone && (nowMs - lastMoveMs) >= 3000) {
        odomSvc.recalibrateBias();
        biasCalDone = true;
      }
    } else {
      lastMoveMs = millis();
      biasCalDone = false;
    }
  }

  midiSvc.update();
  webws.poll();

  vTaskDelay(pdMS_TO_TICKS(2));
}


// app_main() is provided by the arduino-esp32 framework (CONFIG_AUTOSTART_ARDUINO=y).
// It calls Serial.begin(), USB.begin(), initArduino(), then spawns loopTask which
// calls setup() once and loop() forever. No manual entry point needed here.
