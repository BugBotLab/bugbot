#include "ConfigPortalService.h"
#include "ConfigService.h"
#include "../net/WebWS.h"
#include <ESPAsyncWebServer.h>

namespace {
const char kRestartResponse[] = "{\"ok\":true,\"message\":\"saved\",\"rebooting\":true}";
}

String ConfigPortalService::jsonEscape_(const String& s) {
  String out;
  out.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); ++i) {
    const char c = s[i];
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '"': out += "\\\""; break;
      case '\n': out += "\\n"; break;
      case '\r': break;
      case '\t': out += "\\t"; break;
      default: out += c; break;
    }
  }
  return out;
}

String ConfigPortalService::htmlEscape_(const String& s) {
  String out;
  out.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); ++i) {
    const char c = s[i];
    switch (c) {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '"': out += "&quot;"; break;
      case '\'': out += "&#39;"; break;
      default: out += c; break;
    }
  }
  return out;
}

bool ConfigPortalService::getBoolParam_(AsyncWebServerRequest* req, const char* name, bool defaultVal) {
  if (!req->hasParam(name, true)) return defaultVal;
  String v = req->getParam(name, true)->value();
  v.trim();
  v.toLowerCase();
  return (v == "1" || v == "true" || v == "on" || v == "yes");
}

String ConfigPortalService::getStringParam_(AsyncWebServerRequest* req, const char* name, const String& defaultVal) {
  if (!req->hasParam(name, true)) return defaultVal;
  return req->getParam(name, true)->value();
}

uint32_t ConfigPortalService::getUIntParam_(AsyncWebServerRequest* req, const char* name, uint32_t defaultVal) {
  if (!req->hasParam(name, true)) return defaultVal;
  return (uint32_t)req->getParam(name, true)->value().toInt();
}

float ConfigPortalService::getFloatParam_(AsyncWebServerRequest* req, const char* name, float defaultVal) {
  if (!req->hasParam(name, true)) return defaultVal;
  return req->getParam(name, true)->value().toFloat();
}

String ConfigPortalService::buildJson_(const RuntimeConfig& cfg, ConfigService& svc) {
  String s;
  s.reserve(1024);
  s += "{";
  s += "\"wifi\":{";
  s += "\"sta_enabled\":" + String(cfg.wifi.staEnabled ? "true" : "false") + ",";
  s += "\"sta_ssid\":\"" + jsonEscape_(cfg.wifi.staSsid) + "\",";
  s += "\"sta_password\":\"" + jsonEscape_(cfg.wifi.staPassword) + "\",";
  s += "\"ap_enabled\":" + String(cfg.wifi.apEnabled ? "true" : "false") + ",";
  s += "\"ap_ssid\":\"" + jsonEscape_(cfg.wifi.apSsid) + "\",";
  s += "\"ap_password\":\"" + jsonEscape_(cfg.wifi.apPassword) + "\",";
  s += "\"hostname\":\"" + jsonEscape_(cfg.wifi.hostname) + "\",";
  s += "\"mdns_enabled\":" + String(cfg.wifi.mdnsEnabled ? "true" : "false") + ",";
  s += "\"connect_timeout_ms\":" + String(cfg.wifi.connectTimeoutMs) + ",";
  s += "\"wifi_sleep\":" + String(cfg.wifi.wifiSleep ? "true" : "false");
  s += "},";

  s += "\"motion\":{";
  s += "\"field_centric\":" + String(cfg.motion.fieldCentric ? "true" : "false") + ",";
  s += "\"max_linear_speed\":" + String(cfg.motion.maxLinearSpeed, 3) + ",";
  s += "\"max_rot_speed\":" + String(cfg.motion.maxRotSpeed, 3) + ",";
  s += "\"input_deadband\":" + String(cfg.motion.inputDeadband, 3) + ",";
  s += "\"control_rate_hz\":" + String(cfg.motion.controlRateHz) + ",";
  s += "\"teleop_timeout_ms\":" + String(cfg.motion.teleopTimeoutMs) + ",";
  s += "\"heading_hold_enabled\":" + String(cfg.motion.headingHoldEnabled ? "true" : "false") + ",";
  s += "\"heading_kp\":" + String(cfg.motion.headingKp, 6) + ",";
  s += "\"heading_ki\":" + String(cfg.motion.headingKi, 6) + ",";
  s += "\"heading_kd\":" + String(cfg.motion.headingKd, 6) + ",";
  s += "\"slew_limit_enabled\":" + String(cfg.motion.slewLimitEnabled ? "true" : "false") + ",";
  s += "\"slew_rate\":" + String(cfg.motion.slewRate, 3);
  s += "},";

  s += "\"system\":{";
  s += "\"boot_delay_ms\":" + String(cfg.system.bootDelayMs) + ",";
  s += "\"serial_baud\":" + String(cfg.system.serialBaud) + ",";
  s += "\"safe_boot_on_config_error\":" + String(cfg.system.safeBootOnConfigError ? "true" : "false") + ",";
  s += "\"log_to_sd\":" + String(cfg.system.logToSd ? "true" : "false") + ",";
  s += "\"telemetry_rate_hz\":" + String(cfg.system.telemetryRateHz) + ",";
  s += "\"ui_update_rate_hz\":" + String(cfg.system.uiUpdateRateHz) + ",";
  s += "\"start_web_ui_on_boot\":" + String(cfg.system.startWebUiOnBoot ? "true" : "false") + ",";
  s += "\"midi_enabled\":" + String(cfg.system.midiEnabled ? "true" : "false") + ",";
  s += "\"udp_messages_enabled\":" + String(cfg.system.udpMessagesEnabled ? "true" : "false") + ",";
  s += "\"auto_peripheral_power\":" + String(cfg.system.autoPeripheralPower ? "true" : "false") + ",";
  s += "\"peripheral_idle_timeout_ms\":" + String(cfg.system.peripheralIdleTimeoutMs) + ",";
  s += "\"peripheral_power_on_settle_ms\":" + String(cfg.system.peripheralPowerOnSettleMs);
  s += "},";

  s += "\"arena\":{";
  s += "\"width_mm\":"                  + String(cfg.arena.widthMm, 1)               + ",";
  s += "\"height_mm\":"                 + String(cfg.arena.heightMm, 1)              + ",";
  s += "\"tag_center_from_corner_mm\":" + String(cfg.arena.tagCenterFromCornerMm, 1) + ",";
  s += "\"tag_size_mm\":"               + String(cfg.arena.tagSizeMm, 1);
  s += "},";

  s += "\"storage\":{";
  s += "\"total_bytes\":" + String((uint32_t)svc.totalBytes()) + ",";
  s += "\"used_bytes\":" + String((uint32_t)svc.usedBytes());
  s += "}";
  s += "}";
  return s;
}

String ConfigPortalService::buildHtml_(const RuntimeConfig& cfg) {
  String h;
  h.reserve(16000);
  auto checkbox = [](bool v) { return v ? "checked" : ""; };
  h += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>BugBot Config</title>";
  h += "<style>body{font-family:Arial,sans-serif;margin:0;background:#0f172a;color:#e2e8f0}main{max-width:900px;margin:0 auto;padding:20px}fieldset{border:1px solid #334155;border-radius:12px;margin:0 0 18px;padding:16px;background:#111827}legend{padding:0 10px;font-weight:700}label{display:block;margin:10px 0 4px}input[type=text],input[type=password],input[type=number]{width:100%;padding:10px;border-radius:8px;border:1px solid #475569;background:#0b1220;color:#e2e8f0;box-sizing:border-box} .row{display:grid;grid-template-columns:1fr 1fr;gap:14px} .check{display:flex;align-items:center;gap:10px;margin:10px 0} button{padding:12px 18px;border:none;border-radius:10px;background:#2563eb;color:#fff;font-weight:700;cursor:pointer} .muted{color:#94a3b8} .mono{font-family:Consolas,monospace} .ok{color:#86efac} .warn{color:#fbbf24}</style></head><body><main>";
  h += "<h1>BugBot configuration</h1><p class='muted'>Edit settings below, save them to FFat, then the robot will reboot and load the new configuration.</p>";
  h += "<form id='cfgForm' method='post' action='/api/config'>";
  h += "<button type='submit' style='margin-bottom:18px'>Save to FFat and reboot</button> <span id='status' class='muted'></span>";

  h += "<fieldset><legend>Wi-Fi</legend>";
  h += "<label class='check'><input type='checkbox' name='sta_enabled' value='1' " + String(checkbox(cfg.wifi.staEnabled)) + "> Enable STA client</label>";
  h += "<label>STA SSID</label><input type='text' name='sta_ssid' value='" + htmlEscape_(cfg.wifi.staSsid) + "'>";
  h += "<label>STA password</label><input type='password' name='sta_password' value='" + htmlEscape_(cfg.wifi.staPassword) + "'>";
  h += "<label class='check'><input type='checkbox' name='ap_enabled' value='1' " + String(checkbox(cfg.wifi.apEnabled)) + "> Enable access point</label>";
  h += "<div class='row'><div><label>AP SSID</label><input type='text' name='ap_ssid' value='" + htmlEscape_(cfg.wifi.apSsid) + "'></div>";
  h += "<div><label>AP password</label><input type='password' name='ap_password' value='" + htmlEscape_(cfg.wifi.apPassword) + "'></div></div>";
  h += "<div class='row'><div><label>Hostname</label><input type='text' name='hostname' value='" + htmlEscape_(cfg.wifi.hostname) + "'></div>";
  h += "<div><label>Connect timeout (ms)</label><input type='number' name='connect_timeout_ms' min='1000' step='100' value='" + String(cfg.wifi.connectTimeoutMs) + "'></div></div>";
  h += "<label class='check'><input type='checkbox' name='mdns_enabled' value='1' " + String(checkbox(cfg.wifi.mdnsEnabled)) + "> Enable mDNS</label>";
  h += "<label class='check'><input type='checkbox' name='wifi_sleep' value='1' " + String(checkbox(cfg.wifi.wifiSleep)) + "> Enable Wi-Fi power save</label>";
  h += "</fieldset>";

  h += "<fieldset><legend>Motion</legend>";
  h += "<label class='check'><input type='checkbox' name='field_centric' value='1' " + String(checkbox(cfg.motion.fieldCentric)) + "> Field centric</label>";
  h += "<div class='row'><div><label>Max linear speed</label><input type='number' step='0.01' name='max_linear_speed' value='" + String(cfg.motion.maxLinearSpeed, 3) + "'></div>";
  h += "<div><label>Max rotational speed</label><input type='number' step='0.01' name='max_rot_speed' value='" + String(cfg.motion.maxRotSpeed, 3) + "'></div></div>";
  h += "<div class='row'><div><label>Input deadband</label><input type='number' step='0.001' name='input_deadband' value='" + String(cfg.motion.inputDeadband, 3) + "'></div>";
  h += "<div><label>Control rate (Hz)</label><input type='number' step='1' name='control_rate_hz' value='" + String(cfg.motion.controlRateHz) + "'></div></div>";
  h += "<div class='row'><div><label>Teleop timeout (ms)</label><input type='number' step='10' name='teleop_timeout_ms' value='" + String(cfg.motion.teleopTimeoutMs) + "'></div>";
  h += "<div><label>Slew rate</label><input type='number' step='0.01' name='slew_rate' value='" + String(cfg.motion.slewRate, 3) + "'></div></div>";
  h += "<label class='check'><input type='checkbox' name='heading_hold_enabled' value='1' " + String(checkbox(cfg.motion.headingHoldEnabled)) + "> Heading hold enabled</label>";
  h += "<div class='row'><div><label>Heading Kp</label><input type='number' step='0.0001' name='heading_kp' value='" + String(cfg.motion.headingKp, 6) + "'></div>";
  h += "<div><label>Heading Ki</label><input type='number' step='0.0001' name='heading_ki' value='" + String(cfg.motion.headingKi, 6) + "'></div></div>";
  h += "<div class='row'><div><label>Heading Kd</label><input type='number' step='0.0001' name='heading_kd' value='" + String(cfg.motion.headingKd, 6) + "'></div>";
  h += "<div><label class='check'><input type='checkbox' name='slew_limit_enabled' value='1' " + String(checkbox(cfg.motion.slewLimitEnabled)) + "> Slew limit enabled</label></div></div>";
  h += "</fieldset>";

  h += "<fieldset><legend>System</legend>";
  h += "<div class='row'><div><label>Boot delay (ms)</label><input type='number' step='10' name='boot_delay_ms' value='" + String(cfg.system.bootDelayMs) + "'></div>";
  h += "<div><label>Serial baud</label><input type='number' step='1' name='serial_baud' value='" + String(cfg.system.serialBaud) + "'></div></div>";
  h += "<div class='row'><div><label>Telemetry rate (Hz)</label><input type='number' step='1' name='telemetry_rate_hz' value='" + String(cfg.system.telemetryRateHz) + "'></div>";
  h += "<div><label>UI update rate (Hz)</label><input type='number' step='1' name='ui_update_rate_hz' value='" + String(cfg.system.uiUpdateRateHz) + "'></div></div>";
  h += "<label class='check'><input type='checkbox' name='safe_boot_on_config_error' value='1' " + String(checkbox(cfg.system.safeBootOnConfigError)) + "> Safe boot on config error</label>";
  h += "<label class='check'><input type='checkbox' name='log_to_sd' value='1' " + String(checkbox(cfg.system.logToSd)) + "> Log to SD flag (currently informational)</label>";
  h += "<label class='check'><input type='checkbox' name='start_web_ui_on_boot' value='1' " + String(checkbox(cfg.system.startWebUiOnBoot)) + "> Start web UI on boot</label>";
  h += "<label class='check'><input type='checkbox' name='midi_enabled' value='1' " + String(checkbox(cfg.system.midiEnabled)) + "> Enable AppleMIDI</label>";
  h += "<label class='check'><input type='checkbox' name='udp_messages_enabled' value='1' " + String(checkbox(cfg.system.udpMessagesEnabled)) + "> Enable outgoing telemetry / UDP-style messages</label>";
  h += "<label class='check'><input type='checkbox' name='auto_peripheral_power' value='1' " + String(checkbox(cfg.system.autoPeripheralPower)) + "> Auto power-gate motor / servo rail when idle</label>";
  h += "<div class='row'><div><label>Peripheral idle timeout (ms)</label><input type='number' step='10' name='peripheral_idle_timeout_ms' value='" + String(cfg.system.peripheralIdleTimeoutMs) + "'></div>";
  h += "<div><label>Peripheral power-on settle (ms)</label><input type='number' step='1' name='peripheral_power_on_settle_ms' value='" + String(cfg.system.peripheralPowerOnSettleMs) + "'></div></div>";
  h += "</fieldset>";

  h += "<fieldset><legend>Arena</legend>";
  h += "<p class='muted' style='margin:4px 0 12px'>Dimensions of the playfield inner boundary. Tags 0-7 are auto-positioned clockwise from the top-left corner.</p>";
  h += "<div class='row'><div><label>Width (mm)</label><input type='number' step='1' name='arena_width_mm' value='" + String(cfg.arena.widthMm, 1) + "'></div>";
  h += "<div><label>Height (mm)</label><input type='number' step='1' name='arena_height_mm' value='" + String(cfg.arena.heightMm, 1) + "'></div></div>";
  h += "<div class='row'><div><label>Tag centre from corner (mm)</label><input type='number' step='0.5' name='arena_tag_center_mm' value='" + String(cfg.arena.tagCenterFromCornerMm, 1) + "'></div>";
  h += "<div><label>Tag printed size (mm)</label><input type='number' step='1' name='arena_tag_size_mm' value='" + String(cfg.arena.tagSizeMm, 1) + "'></div></div>";
  h += "</fieldset>";

  h += "<button type='submit'>Save to FFat and reboot</button> <span id='status' class='muted'></span></form>";
  h += "<p class='muted'>Config page: <span class='mono'>/config</span>. After saving, reconnect to the AP if its SSID or password changed.</p>";
  h += R"JS(<script>
const form=document.getElementById('cfgForm');
const status=document.getElementById('status');
form.addEventListener('submit', async (e)=>{
  e.preventDefault();
  status.textContent='Saving...';
  const fd=new FormData(form);
  for(const cb of form.querySelectorAll('input[type=checkbox]')){
    if(!fd.has(cb.name)) fd.append(cb.name,'0');
  }
  try{
    const res=await fetch('/api/config',{method:'POST',body:new URLSearchParams(fd)});
    const txt=await res.text();
    if(!res.ok){ status.textContent='Save failed: '+txt; status.className='warn'; return; }
    status.textContent='Saved. Rebooting...'; status.className='ok';
  }catch(err){ status.textContent='Save failed: '+err; status.className='warn'; }
});
</script>)JS";
  h += "</main></body></html>";
  return h;
}

void ConfigPortalService::scheduleRestart_() {
  xTaskCreate([](void*) {
    vTaskDelay(pdMS_TO_TICKS(1200));
    ESP.restart();
  }, "cfg_reboot", 2048, nullptr, 1, nullptr);
}

void ConfigPortalService::attach(WebWS& webws, ConfigService& configSvc, RuntimeConfig& runtimeCfg) {
  AsyncWebServer& server = webws.server();

  server.on("/config", HTTP_GET, [&runtimeCfg](AsyncWebServerRequest* req) {
    req->send(200, "text/html", buildHtml_(runtimeCfg));
  });

  server.on("/api/config", HTTP_GET, [&runtimeCfg, &configSvc](AsyncWebServerRequest* req) {
    req->send(200, "application/json", buildJson_(runtimeCfg, configSvc));
  });

  server.on("/api/config", HTTP_POST, [&runtimeCfg, &configSvc](AsyncWebServerRequest* req) {
    RuntimeConfig next = runtimeCfg;

    next.wifi.staEnabled = getBoolParam_(req, "sta_enabled", false);
    next.wifi.staSsid = getStringParam_(req, "sta_ssid", "");
    next.wifi.staPassword = getStringParam_(req, "sta_password", "");
    next.wifi.apEnabled = getBoolParam_(req, "ap_enabled", true);
    next.wifi.apSsid = getStringParam_(req, "ap_ssid", next.wifi.apSsid);
    next.wifi.apPassword = getStringParam_(req, "ap_password", next.wifi.apPassword);
    next.wifi.hostname = getStringParam_(req, "hostname", next.wifi.hostname);
    next.wifi.mdnsEnabled = getBoolParam_(req, "mdns_enabled", true);
    next.wifi.connectTimeoutMs = getUIntParam_(req, "connect_timeout_ms", next.wifi.connectTimeoutMs);
    next.wifi.wifiSleep = getBoolParam_(req, "wifi_sleep", false);

    next.motion.fieldCentric = getBoolParam_(req, "field_centric", true);
    next.motion.maxLinearSpeed = getFloatParam_(req, "max_linear_speed", next.motion.maxLinearSpeed);
    next.motion.maxRotSpeed = getFloatParam_(req, "max_rot_speed", next.motion.maxRotSpeed);
    next.motion.inputDeadband = getFloatParam_(req, "input_deadband", next.motion.inputDeadband);
    next.motion.controlRateHz = getUIntParam_(req, "control_rate_hz", next.motion.controlRateHz);
    next.motion.teleopTimeoutMs = getUIntParam_(req, "teleop_timeout_ms", next.motion.teleopTimeoutMs);
    next.motion.headingHoldEnabled = getBoolParam_(req, "heading_hold_enabled", next.motion.headingHoldEnabled);
    next.motion.headingKp = getFloatParam_(req, "heading_kp", next.motion.headingKp);
    next.motion.headingKi = getFloatParam_(req, "heading_ki", next.motion.headingKi);
    next.motion.headingKd = getFloatParam_(req, "heading_kd", next.motion.headingKd);
    next.motion.slewLimitEnabled = getBoolParam_(req, "slew_limit_enabled", next.motion.slewLimitEnabled);
    next.motion.slewRate = getFloatParam_(req, "slew_rate", next.motion.slewRate);

    next.system.bootDelayMs = getUIntParam_(req, "boot_delay_ms", next.system.bootDelayMs);
    next.system.serialBaud = getUIntParam_(req, "serial_baud", next.system.serialBaud);
    next.system.safeBootOnConfigError = getBoolParam_(req, "safe_boot_on_config_error", next.system.safeBootOnConfigError);
    next.system.logToSd = getBoolParam_(req, "log_to_sd", next.system.logToSd);
    next.system.telemetryRateHz = getUIntParam_(req, "telemetry_rate_hz", next.system.telemetryRateHz);
    next.system.uiUpdateRateHz = getUIntParam_(req, "ui_update_rate_hz", next.system.uiUpdateRateHz);
    next.system.startWebUiOnBoot = getBoolParam_(req, "start_web_ui_on_boot", next.system.startWebUiOnBoot);
    next.system.midiEnabled = getBoolParam_(req, "midi_enabled", next.system.midiEnabled);
    next.system.udpMessagesEnabled = getBoolParam_(req, "udp_messages_enabled", next.system.udpMessagesEnabled);
    next.system.autoPeripheralPower = getBoolParam_(req, "auto_peripheral_power", next.system.autoPeripheralPower);
    next.system.peripheralIdleTimeoutMs = getUIntParam_(req, "peripheral_idle_timeout_ms", next.system.peripheralIdleTimeoutMs);
    next.system.peripheralPowerOnSettleMs = getUIntParam_(req, "peripheral_power_on_settle_ms", next.system.peripheralPowerOnSettleMs);

    next.arena.widthMm               = getFloatParam_(req, "arena_width_mm",      next.arena.widthMm);
    next.arena.heightMm              = getFloatParam_(req, "arena_height_mm",     next.arena.heightMm);
    next.arena.tagCenterFromCornerMm = getFloatParam_(req, "arena_tag_center_mm", next.arena.tagCenterFromCornerMm);
    next.arena.tagSizeMm             = getFloatParam_(req, "arena_tag_size_mm",   next.arena.tagSizeMm);

    if (next.arena.widthMm  < 100.0f) next.arena.widthMm  = 100.0f;
    if (next.arena.heightMm < 100.0f) next.arena.heightMm = 100.0f;

    if (next.wifi.staSsid.isEmpty()) {
      next.wifi.staEnabled = false;
    }
    if (next.wifi.apSsid.isEmpty()) next.wifi.apSsid = "bugbot_setup";
    if (!next.wifi.apEnabled) next.wifi.apEnabled = true; // keep recovery path available
    if (next.wifi.apPassword.length() < 8) next.wifi.apPassword = "bugbot123";
    if (next.wifi.hostname.isEmpty()) next.wifi.hostname = "bugbot";

    if (!configSvc.saveAll(next)) {
      req->send(500, "text/plain", "Failed to save config to FFat");
      return;
    }

    runtimeCfg = next;
    req->send(200, "application/json", kRestartResponse);
    scheduleRestart_();
  });
}
