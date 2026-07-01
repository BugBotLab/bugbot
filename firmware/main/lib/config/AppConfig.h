// AppConfig.h
// Runtime configuration structs loaded from LittleFS by ConfigService.
// RuntimeConfig is the top-level aggregate; ConfigPortalService exposes it via HTTP.
#pragma once
#include <Arduino.h>

struct WifiRuntimeConfig {
  bool staEnabled = false;
  String staSsid = "";
  String staPassword = "";

  bool apEnabled = true;
  String apSsid = "bugbot_setup";
  String apPassword = "bugbot123";

  String hostname = "bugbot";
  bool mdnsEnabled = true;
  uint32_t connectTimeoutMs = 15000;
  bool wifiSleep = true;
};

struct MotionRuntimeConfig {
  bool fieldCentric = true;
  float maxLinearSpeed = 1.0f;
  float maxRotSpeed = 1.0f;
  float inputDeadband = 0.05f;
  uint32_t controlRateHz = 50;
  uint32_t teleopTimeoutMs = 500;
  bool headingHoldEnabled = true;
  float headingKp = 0.010f;
  float headingKi = 0.0f;
  float headingKd = 0.020f;
  bool slewLimitEnabled = true;
  float slewRate = 2.0f;
};

struct SystemRuntimeConfig {
  uint32_t bootDelayMs = 300;
  uint32_t serialBaud = 921600;
  bool safeBootOnConfigError = true;
  bool logToSd = false;
  uint32_t telemetryRateHz = 5;
  uint32_t uiUpdateRateHz = 10;
  bool startWebUiOnBoot = true;
  bool midiEnabled = true;
  bool udpMessagesEnabled = true;
  bool autoPeripheralPower = true;
  uint32_t peripheralIdleTimeoutMs = 1200;
  uint32_t peripheralPowerOnSettleMs = 12;

  // ── ESP-NOW mode ──────────────────────────────────────────────────────────
  // When espnowEnabled is true, ESP-NOW is the SOLE comms transport and all
  // WiFi-based features (STA/AP/WebSocket/HTTP portal/mDNS/MIDI/UDP) are skipped.
  // Defaults keep a normal boot identical to today (WiFi mode).
  bool    espnowEnabled = false;
  String  deviceId      = "";   // 16 hex chars (64-bit). Empty until provisioned.
  String  pairingKey    = "";   // 16 hex chars (64-bit) secret. Empty until provisioned.
  uint8_t espnowChannel = 1;
};

// Arena tag layout — 2 tags per wall at 1/3 and 2/3, clockwise from top-left (IDs 0-7):
//
//   (0,H) ──0──────1── (W,H)
//    │                   │
//    7                   2
//    │                   │
//    6                   3
//    │                   │
//   (0,0) ──5──────4── (W,0)
//
//  Tag face convention: degrees CCW from +X (right wall = 180°, top wall = 270°).
//  tagCenterFromCornerMm is retained for config-file compatibility but unused by layout.
struct ArenaRuntimeConfig {
  float widthMm               = 555.0f;  // inner arena width  (mm)
  float heightMm              = 355.0f;  // inner arena height (mm)
  float tagCenterFromCornerMm = 33.5f;   // legacy — not used by current layout
  float tagSizeMm             = 50.0f;   // printed tag size (mm)
};

struct RuntimeConfig {
  WifiRuntimeConfig   wifi{};
  MotionRuntimeConfig motion{};
  SystemRuntimeConfig system{};
  ArenaRuntimeConfig  arena{};
};

inline RuntimeConfig makeDefaultRuntimeConfig() {
  return RuntimeConfig{};
}
