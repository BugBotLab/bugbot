#include "ConfigService.h"

namespace {
String boolStr_(bool v) { return v ? "1" : "0"; }
}

String ConfigService::trimCopy_(const String& s) {
  String out = s;
  out.trim();
  return out;
}

bool ConfigService::toBool_(const String& s, bool defaultValue) {
  String v = trimCopy_(s);
  v.toLowerCase();
  if (v == "1" || v == "true" || v == "yes" || v == "on") return true;
  if (v == "0" || v == "false" || v == "no" || v == "off") return false;
  return defaultValue;
}

bool ConfigService::beginFs_() {
  if (fsMounted_) return true;

  if (!LittleFS.begin(true, "/littlefs", 10, "spiffs")) {
    Serial.println("[CONFIG] LittleFS mount failed");
    return false;
  }

  fsMounted_ = true;
  if (!LittleFS.exists("/config")) {
    LittleFS.mkdir("/config");
  }

  return true;
}

bool ConfigService::beginAndLoad(RuntimeConfig& cfg) {
  if (!beginFs_()) {
    Serial.println("[CONFIG] LittleFS unavailable, using defaults");
    return false;
  }

  ensureDefaultsExist();
  return loadAll_(cfg);
}

bool ConfigService::reload(RuntimeConfig& cfg) {
  if (!beginFs_()) {
    Serial.println("[CONFIG] LittleFS reload failed, FS unavailable");
    return false;
  }
  return loadAll_(cfg);
}

bool ConfigService::ensureDefaultsExist() {
  if (!beginFs_()) return false;

  RuntimeConfig defaults = makeDefaultRuntimeConfig();
  bool changed = false;

  if (!LittleFS.exists("/config/wifi.cfg")) {
    changed = saveText_(
      "/config/wifi.cfg",
      String("# Wi-Fi configuration\n") +
      "sta_enabled=" + boolStr_(defaults.wifi.staEnabled) + "\n" +
      "sta_ssid=" + defaults.wifi.staSsid + "\n" +
      "sta_password=" + defaults.wifi.staPassword + "\n\n" +
      "ap_enabled=" + boolStr_(defaults.wifi.apEnabled) + "\n" +
      "ap_ssid=" + defaults.wifi.apSsid + "\n" +
      "ap_password=" + defaults.wifi.apPassword + "\n\n" +
      "hostname=" + defaults.wifi.hostname + "\n" +
      "mdns_enabled=" + boolStr_(defaults.wifi.mdnsEnabled) + "\n" +
      "connect_timeout_ms=" + String(defaults.wifi.connectTimeoutMs) + "\n" +
      "wifi_sleep=" + boolStr_(defaults.wifi.wifiSleep) + "\n"
    ) || changed;
  }

  if (!LittleFS.exists("/config/motion.cfg")) {
    changed = saveText_(
      "/config/motion.cfg",
      String("# Motion configuration\n") +
      "field_centric=" + boolStr_(defaults.motion.fieldCentric) + "\n" +
      "max_linear_speed=" + String(defaults.motion.maxLinearSpeed, 3) + "\n" +
      "max_rot_speed=" + String(defaults.motion.maxRotSpeed, 3) + "\n" +
      "input_deadband=" + String(defaults.motion.inputDeadband, 3) + "\n" +
      "control_rate_hz=" + String(defaults.motion.controlRateHz) + "\n" +
      "teleop_timeout_ms=" + String(defaults.motion.teleopTimeoutMs) + "\n" +
      "heading_hold_enabled=" + boolStr_(defaults.motion.headingHoldEnabled) + "\n" +
      "heading_kp=" + String(defaults.motion.headingKp, 6) + "\n" +
      "heading_ki=" + String(defaults.motion.headingKi, 6) + "\n" +
      "heading_kd=" + String(defaults.motion.headingKd, 6) + "\n" +
      "slew_limit_enabled=" + boolStr_(defaults.motion.slewLimitEnabled) + "\n" +
      "slew_rate=" + String(defaults.motion.slewRate, 3) + "\n"
    ) || changed;
  }

  if (!LittleFS.exists("/config/arena.cfg")) {
    changed = saveText_(
      "/config/arena.cfg",
      String("# Arena dimensions and tag layout\n") +
      "width_mm=" + String(defaults.arena.widthMm, 1) + "\n" +
      "height_mm=" + String(defaults.arena.heightMm, 1) + "\n" +
      "tag_center_from_corner_mm=" + String(defaults.arena.tagCenterFromCornerMm, 1) + "\n" +
      "tag_size_mm=" + String(defaults.arena.tagSizeMm, 1) + "\n"
    ) || changed;
  }

  if (!LittleFS.exists("/config/system.cfg")) {
    changed = saveText_(
      "/config/system.cfg",
      String("# System configuration\n") +
      "boot_delay_ms=" + String(defaults.system.bootDelayMs) + "\n" +
      "serial_baud=" + String(defaults.system.serialBaud) + "\n" +
      "safe_boot_on_config_error=" + boolStr_(defaults.system.safeBootOnConfigError) + "\n" +
      "log_to_sd=" + boolStr_(defaults.system.logToSd) + "\n" +
      "telemetry_rate_hz=" + String(defaults.system.telemetryRateHz) + "\n" +
      "ui_update_rate_hz=" + String(defaults.system.uiUpdateRateHz) + "\n" +
      "start_web_ui_on_boot=" + boolStr_(defaults.system.startWebUiOnBoot) + "\n" +
      "midi_enabled=" + boolStr_(defaults.system.midiEnabled) + "\n" +
      "udp_messages_enabled=" + boolStr_(defaults.system.udpMessagesEnabled) + "\n" +
      "auto_peripheral_power=" + boolStr_(defaults.system.autoPeripheralPower) + "\n" +
      "peripheral_idle_timeout_ms=" + String(defaults.system.peripheralIdleTimeoutMs) + "\n" +
      "peripheral_power_on_settle_ms=" + String(defaults.system.peripheralPowerOnSettleMs) + "\n" +
      "espnow_enabled=" + boolStr_(defaults.system.espnowEnabled) + "\n" +
      "espnow_channel=" + String(defaults.system.espnowChannel) + "\n" +
      "device_id=" + defaults.system.deviceId + "\n" +
      "pairing_key=" + defaults.system.pairingKey + "\n"
    ) || changed;
  }

  return changed || true;
}

bool ConfigService::loadAll_(RuntimeConfig& cfg) {
  if (!LittleFS.exists("/config")) {
    Serial.println("[CONFIG] /config folder not found; using defaults");
    return false;
  }

  bool wifiLoaded   = loadKeyValueFile_("/config/wifi.cfg",   handleWifi_,   cfg);
  bool motionLoaded = loadKeyValueFile_("/config/motion.cfg", handleMotion_, cfg);
  bool systemLoaded = loadKeyValueFile_("/config/system.cfg", handleSystem_, cfg);
  bool arenaLoaded  = loadKeyValueFile_("/config/arena.cfg",  handleArena_,  cfg);

  if (cfg.wifi.staSsid.isEmpty()) {
    cfg.wifi.staEnabled = false;
  }

  Serial.printf("[CONFIG] wifi=%d motion=%d system=%d arena=%d\n",
                wifiLoaded ? 1 : 0,
                motionLoaded ? 1 : 0,
                systemLoaded ? 1 : 0,
                arenaLoaded ? 1 : 0);

  return wifiLoaded || motionLoaded || systemLoaded;
}

bool ConfigService::loadKeyValueFile_(const char* path,
                                      void (*handler)(const String&, const String&, RuntimeConfig&),
                                      RuntimeConfig& cfg) {
  File file = LittleFS.open(path, FILE_READ);
  if (!file) {
    Serial.printf("[CONFIG] Missing file: %s\n", path);
    return false;
  }

  if (file.size() >= 2) {
    int b0 = file.read();
    int b1 = file.read();
    file.seek(0);
    if ((b0 == 0xFF && b1 == 0xFE) || (b0 == 0xFE && b1 == 0xFF)) {
      Serial.printf("[CONFIG] ERROR: %s looks like UTF-16. Save as UTF-8/plain text.\n", path);
      file.close();
      return false;
    }
  }

  Serial.printf("[CONFIG] Loading %s\n", path);
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.isEmpty()) continue;
    if (line.startsWith("#") || line.startsWith(";") || line.startsWith("//")) continue;

    const int eq = line.indexOf('=');
    if (eq < 0) {
      Serial.printf("[CONFIG] Skipping malformed line in %s: %s\n", path, line.c_str());
      continue;
    }

    const String key = trimCopy_(line.substring(0, eq));
    const String value = trimCopy_(line.substring(eq + 1));
    if (key.isEmpty()) continue;
    handler(key, value, cfg);
  }

  file.close();
  return true;
}

bool ConfigService::saveText_(const char* path, const String& content) {
  File file = LittleFS.open(path, FILE_WRITE);
  if (!file) {
    Serial.printf("[CONFIG] Failed to open %s for write\n", path);
    return false;
  }

  const size_t written = file.print(content);
  file.close();
  if (written != content.length()) {
    Serial.printf("[CONFIG] Short write on %s\n", path);
    return false;
  }
  return true;
}

bool ConfigService::saveAll(const RuntimeConfig& cfg) {
  if (!beginFs_()) return false;
  if (!LittleFS.exists("/config")) {
    LittleFS.mkdir("/config");
  }

  const String wifiText =
    String("# Wi-Fi configuration\n") +
    "sta_enabled=" + boolStr_(cfg.wifi.staEnabled) + "\n" +
    "sta_ssid=" + cfg.wifi.staSsid + "\n" +
    "sta_password=" + cfg.wifi.staPassword + "\n\n" +
    "ap_enabled=" + boolStr_(cfg.wifi.apEnabled) + "\n" +
    "ap_ssid=" + cfg.wifi.apSsid + "\n" +
    "ap_password=" + cfg.wifi.apPassword + "\n\n" +
    "hostname=" + cfg.wifi.hostname + "\n" +
    "mdns_enabled=" + boolStr_(cfg.wifi.mdnsEnabled) + "\n" +
    "connect_timeout_ms=" + String(cfg.wifi.connectTimeoutMs) + "\n" +
    "wifi_sleep=" + boolStr_(cfg.wifi.wifiSleep) + "\n";

  const String motionText =
    String("# Motion configuration\n") +
    "field_centric=" + boolStr_(cfg.motion.fieldCentric) + "\n" +
    "max_linear_speed=" + String(cfg.motion.maxLinearSpeed, 3) + "\n" +
    "max_rot_speed=" + String(cfg.motion.maxRotSpeed, 3) + "\n" +
    "input_deadband=" + String(cfg.motion.inputDeadband, 3) + "\n" +
    "control_rate_hz=" + String(cfg.motion.controlRateHz) + "\n" +
    "teleop_timeout_ms=" + String(cfg.motion.teleopTimeoutMs) + "\n" +
    "heading_hold_enabled=" + boolStr_(cfg.motion.headingHoldEnabled) + "\n" +
    "heading_kp=" + String(cfg.motion.headingKp, 6) + "\n" +
    "heading_ki=" + String(cfg.motion.headingKi, 6) + "\n" +
    "heading_kd=" + String(cfg.motion.headingKd, 6) + "\n" +
    "slew_limit_enabled=" + boolStr_(cfg.motion.slewLimitEnabled) + "\n" +
    "slew_rate=" + String(cfg.motion.slewRate, 3) + "\n";

  const String systemText =
    String("# System configuration\n") +
    "boot_delay_ms=" + String(cfg.system.bootDelayMs) + "\n" +
    "serial_baud=" + String(cfg.system.serialBaud) + "\n" +
    "safe_boot_on_config_error=" + boolStr_(cfg.system.safeBootOnConfigError) + "\n" +
    "log_to_sd=" + boolStr_(cfg.system.logToSd) + "\n" +
    "telemetry_rate_hz=" + String(cfg.system.telemetryRateHz) + "\n" +
    "ui_update_rate_hz=" + String(cfg.system.uiUpdateRateHz) + "\n" +
    "start_web_ui_on_boot=" + boolStr_(cfg.system.startWebUiOnBoot) + "\n" +
    "midi_enabled=" + boolStr_(cfg.system.midiEnabled) + "\n" +
    "udp_messages_enabled=" + boolStr_(cfg.system.udpMessagesEnabled) + "\n" +
    "auto_peripheral_power=" + boolStr_(cfg.system.autoPeripheralPower) + "\n" +
    "peripheral_idle_timeout_ms=" + String(cfg.system.peripheralIdleTimeoutMs) + "\n" +
    "peripheral_power_on_settle_ms=" + String(cfg.system.peripheralPowerOnSettleMs) + "\n" +
    "espnow_enabled=" + boolStr_(cfg.system.espnowEnabled) + "\n" +
    "espnow_channel=" + String(cfg.system.espnowChannel) + "\n" +
    "device_id=" + cfg.system.deviceId + "\n" +
    "pairing_key=" + cfg.system.pairingKey + "\n";

  const String arenaText =
    String("# Arena dimensions and tag layout\n") +
    "width_mm=" + String(cfg.arena.widthMm, 1) + "\n" +
    "height_mm=" + String(cfg.arena.heightMm, 1) + "\n" +
    "tag_center_from_corner_mm=" + String(cfg.arena.tagCenterFromCornerMm, 1) + "\n" +
    "tag_size_mm=" + String(cfg.arena.tagSizeMm, 1) + "\n";

  const bool ok =
    saveText_("/config/wifi.cfg",   wifiText)   &&
    saveText_("/config/motion.cfg", motionText) &&
    saveText_("/config/system.cfg", systemText) &&
    saveText_("/config/arena.cfg",  arenaText);

  if (ok) {
    Serial.println("[CONFIG] Saved all config files to LittleFS");
  }
  return ok;
}

size_t ConfigService::totalBytes() const {
  return fsMounted_ ? LittleFS.totalBytes() : 0;
}

size_t ConfigService::usedBytes() const {
  return fsMounted_ ? LittleFS.usedBytes() : 0;
}

void ConfigService::handleArena_(const String& key, const String& value, RuntimeConfig& cfg) {
  if      (key == "width_mm")                  cfg.arena.widthMm               = value.toFloat();
  else if (key == "height_mm")                 cfg.arena.heightMm              = value.toFloat();
  else if (key == "tag_center_from_corner_mm") cfg.arena.tagCenterFromCornerMm = value.toFloat();
  else if (key == "tag_size_mm")               cfg.arena.tagSizeMm             = value.toFloat();
  else Serial.printf("[CONFIG][arena] Unknown key: %s\n", key.c_str());
}

void ConfigService::handleWifi_(const String& key, const String& value, RuntimeConfig& cfg) {
  if (key == "sta_enabled") cfg.wifi.staEnabled = toBool_(value, cfg.wifi.staEnabled);
  else if (key == "sta_ssid") cfg.wifi.staSsid = value;
  else if (key == "sta_password") cfg.wifi.staPassword = value;
  else if (key == "ap_enabled") cfg.wifi.apEnabled = toBool_(value, cfg.wifi.apEnabled);
  else if (key == "ap_ssid") cfg.wifi.apSsid = value;
  else if (key == "ap_password") cfg.wifi.apPassword = value;
  else if (key == "hostname") cfg.wifi.hostname = value;
  else if (key == "mdns_enabled") cfg.wifi.mdnsEnabled = toBool_(value, cfg.wifi.mdnsEnabled);
  else if (key == "connect_timeout_ms") cfg.wifi.connectTimeoutMs = (uint32_t)value.toInt();
  else if (key == "wifi_sleep") cfg.wifi.wifiSleep = toBool_(value, cfg.wifi.wifiSleep);
  else Serial.printf("[CONFIG][wifi] Unknown key: %s\n", key.c_str());
}

void ConfigService::handleMotion_(const String& key, const String& value, RuntimeConfig& cfg) {
  if (key == "field_centric") cfg.motion.fieldCentric = toBool_(value, cfg.motion.fieldCentric);
  else if (key == "max_linear_speed") cfg.motion.maxLinearSpeed = value.toFloat();
  else if (key == "max_rot_speed") cfg.motion.maxRotSpeed = value.toFloat();
  else if (key == "input_deadband") cfg.motion.inputDeadband = value.toFloat();
  else if (key == "control_rate_hz") cfg.motion.controlRateHz = (uint32_t)value.toInt();
  else if (key == "teleop_timeout_ms") cfg.motion.teleopTimeoutMs = (uint32_t)value.toInt();
  else if (key == "heading_hold_enabled") cfg.motion.headingHoldEnabled = toBool_(value, cfg.motion.headingHoldEnabled);
  else if (key == "heading_kp") cfg.motion.headingKp = value.toFloat();
  else if (key == "heading_ki") cfg.motion.headingKi = value.toFloat();
  else if (key == "heading_kd") cfg.motion.headingKd = value.toFloat();
  else if (key == "slew_limit_enabled") cfg.motion.slewLimitEnabled = toBool_(value, cfg.motion.slewLimitEnabled);
  else if (key == "slew_rate") cfg.motion.slewRate = value.toFloat();
  else Serial.printf("[CONFIG][motion] Unknown key: %s\n", key.c_str());
}

void ConfigService::handleSystem_(const String& key, const String& value, RuntimeConfig& cfg) {
  if (key == "boot_delay_ms") cfg.system.bootDelayMs = (uint32_t)value.toInt();
  else if (key == "serial_baud") cfg.system.serialBaud = (uint32_t)value.toInt();
  else if (key == "safe_boot_on_config_error") cfg.system.safeBootOnConfigError = toBool_(value, cfg.system.safeBootOnConfigError);
  else if (key == "log_to_sd") cfg.system.logToSd = toBool_(value, cfg.system.logToSd);
  else if (key == "telemetry_rate_hz") cfg.system.telemetryRateHz = (uint32_t)value.toInt();
  else if (key == "ui_update_rate_hz") cfg.system.uiUpdateRateHz = (uint32_t)value.toInt();
  else if (key == "start_web_ui_on_boot") cfg.system.startWebUiOnBoot = toBool_(value, cfg.system.startWebUiOnBoot);
  else if (key == "midi_enabled") cfg.system.midiEnabled = toBool_(value, cfg.system.midiEnabled);
  else if (key == "udp_messages_enabled") cfg.system.udpMessagesEnabled = toBool_(value, cfg.system.udpMessagesEnabled);
  else if (key == "auto_peripheral_power") cfg.system.autoPeripheralPower = toBool_(value, cfg.system.autoPeripheralPower);
  else if (key == "peripheral_idle_timeout_ms") cfg.system.peripheralIdleTimeoutMs = (uint32_t)value.toInt();
  else if (key == "peripheral_power_on_settle_ms") cfg.system.peripheralPowerOnSettleMs = (uint32_t)value.toInt();
  else if (key == "espnow_enabled") cfg.system.espnowEnabled = toBool_(value, cfg.system.espnowEnabled);
  else if (key == "espnow_channel") cfg.system.espnowChannel = (uint8_t)value.toInt();
  else if (key == "device_id") cfg.system.deviceId = value;
  else if (key == "pairing_key") cfg.system.pairingKey = value;  // loaded from disk only; not a live setter
  else Serial.printf("[CONFIG][system] Unknown key: %s\n", key.c_str());
}

bool ConfigService::setKey(const String& key, const String& value, RuntimeConfig& cfg) {
  // wifi
  if (key == "sta_enabled")              { cfg.wifi.staEnabled        = toBool_(value, cfg.wifi.staEnabled);        return true; }
  if (key == "sta_ssid")                 { cfg.wifi.staSsid           = value;                                      return true; }
  if (key == "sta_password")             { cfg.wifi.staPassword       = value;                                      return true; }
  if (key == "ap_enabled")               { cfg.wifi.apEnabled         = toBool_(value, cfg.wifi.apEnabled);         return true; }
  if (key == "ap_ssid")                  { cfg.wifi.apSsid            = value;                                      return true; }
  if (key == "ap_password")              { cfg.wifi.apPassword        = value;                                      return true; }
  if (key == "hostname")                 { cfg.wifi.hostname          = value;                                      return true; }
  if (key == "mdns_enabled")             { cfg.wifi.mdnsEnabled       = toBool_(value, cfg.wifi.mdnsEnabled);       return true; }
  if (key == "connect_timeout_ms")       { cfg.wifi.connectTimeoutMs  = (uint32_t)value.toInt();                    return true; }
  if (key == "wifi_sleep")               { cfg.wifi.wifiSleep         = toBool_(value, cfg.wifi.wifiSleep);         return true; }
  // motion
  if (key == "field_centric")            { cfg.motion.fieldCentric          = toBool_(value, cfg.motion.fieldCentric);          return true; }
  if (key == "max_linear_speed")         { cfg.motion.maxLinearSpeed        = value.toFloat();                                   return true; }
  if (key == "max_rot_speed")            { cfg.motion.maxRotSpeed           = value.toFloat();                                   return true; }
  if (key == "input_deadband")           { cfg.motion.inputDeadband         = value.toFloat();                                   return true; }
  if (key == "control_rate_hz")          { cfg.motion.controlRateHz         = (uint32_t)value.toInt();                          return true; }
  if (key == "teleop_timeout_ms")        { cfg.motion.teleopTimeoutMs       = (uint32_t)value.toInt();                          return true; }
  if (key == "heading_hold_enabled")     { cfg.motion.headingHoldEnabled    = toBool_(value, cfg.motion.headingHoldEnabled);    return true; }
  if (key == "heading_kp")               { cfg.motion.headingKp             = value.toFloat();                                   return true; }
  if (key == "heading_ki")               { cfg.motion.headingKi             = value.toFloat();                                   return true; }
  if (key == "heading_kd")               { cfg.motion.headingKd             = value.toFloat();                                   return true; }
  if (key == "slew_limit_enabled")       { cfg.motion.slewLimitEnabled      = toBool_(value, cfg.motion.slewLimitEnabled);      return true; }
  if (key == "slew_rate")                { cfg.motion.slewRate              = value.toFloat();                                   return true; }
  // system
  if (key == "boot_delay_ms")            { cfg.system.bootDelayMs           = (uint32_t)value.toInt();                          return true; }
  if (key == "serial_baud")              { cfg.system.serialBaud            = (uint32_t)value.toInt();                          return true; }
  if (key == "safe_boot_on_config_error"){ cfg.system.safeBootOnConfigError = toBool_(value, cfg.system.safeBootOnConfigError); return true; }
  if (key == "log_to_sd")               { cfg.system.logToSd               = toBool_(value, cfg.system.logToSd);               return true; }
  if (key == "telemetry_rate_hz")        { cfg.system.telemetryRateHz       = (uint32_t)value.toInt();                          return true; }
  if (key == "ui_update_rate_hz")        { cfg.system.uiUpdateRateHz        = (uint32_t)value.toInt();                          return true; }
  if (key == "start_web_ui_on_boot")     { cfg.system.startWebUiOnBoot      = toBool_(value, cfg.system.startWebUiOnBoot);      return true; }
  if (key == "midi_enabled")             { cfg.system.midiEnabled           = toBool_(value, cfg.system.midiEnabled);           return true; }
  if (key == "udp_messages_enabled")     { cfg.system.udpMessagesEnabled    = toBool_(value, cfg.system.udpMessagesEnabled);    return true; }
  if (key == "auto_peripheral_power")    { cfg.system.autoPeripheralPower   = toBool_(value, cfg.system.autoPeripheralPower);   return true; }
  if (key == "peripheral_idle_timeout_ms")  { cfg.system.peripheralIdleTimeoutMs  = (uint32_t)value.toInt(); return true; }
  if (key == "peripheral_power_on_settle_ms"){ cfg.system.peripheralPowerOnSettleMs = (uint32_t)value.toInt(); return true; }
  // espnow (pairing_key is intentionally NOT a plain setter — see espnow_init / regenerate_key)
  if (key == "espnow_enabled")           { cfg.system.espnowEnabled         = toBool_(value, cfg.system.espnowEnabled);    return true; }
  if (key == "espnow_channel")           { cfg.system.espnowChannel         = (uint8_t)value.toInt();                       return true; }
  if (key == "device_id")                { cfg.system.deviceId              = value;                                        return true; }
  // arena
  if (key == "width_mm")                 { cfg.arena.widthMm               = value.toFloat(); return true; }
  if (key == "height_mm")                { cfg.arena.heightMm              = value.toFloat(); return true; }
  if (key == "tag_center_from_corner_mm"){ cfg.arena.tagCenterFromCornerMm = value.toFloat(); return true; }
  if (key == "tag_size_mm")              { cfg.arena.tagSizeMm             = value.toFloat(); return true; }
  return false;
}
