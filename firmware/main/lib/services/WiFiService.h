// WiFiService.h
// Manages Wi-Fi bring-up (STA/AP modes) and starts the WebWS HTTP/WebSocket server.
// startWithTelemetry() also spawns a FreeRTOS TX task that streams pose and LiDAR
// packets to connected clients at configurable rates.
#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "../drivers/BoardPowerLib.h"

// Forward decls
class WebWS;
class PoseBus;
class LidarService;
class MotionService;

class WiFiService {
public:
  // Basic Wi-Fi configuration (call before start*)
  void configure(bool sta_enabled, bool ap_enabled,
                 const char* sta_ssid, const char* sta_pass,
                 bool wifi_sleep = false,
                 const char* ap_ssid = "BugBot",
                 const char* ap_pass = "changeme123",
                 uint32_t sta_timeout_ms = 15000);

  void setHostname(const char* hn);
  void setTxPower(wifi_power_t pwr = WIFI_POWER_15dBm);

  // Start Wi-Fi + WS server only
  bool start(WebWS& ws);

  // Start Wi-Fi + WS + Telemetry TX task + Control callback
  bool startWithTelemetry(WebWS& ws,
                          PoseBus& pose,
                          LidarService& lidarSvc,
                          MotionService& motionSvc,
                          BoardPowerLib& boardPower,
                          uint32_t pose_hz  = 10,
                          uint32_t lidar_hz = 5,
                          uint32_t tx_stack_kb = 12,
                          UBaseType_t tx_prio = 2,
                          BaseType_t tx_core = 1);

  // Stop Wi-Fi cleanly for low-power mode
  void stop();

  // Info
  bool       isSTA() const { return sta_enabled_; }
  bool       isAP()  const { return ap_enabled_; }
  bool       isUp()  const { return started_ && (WiFi.status() == WL_CONNECTED || WiFi.getMode() == WIFI_MODE_AP || WiFi.getMode() == WIFI_MODE_APSTA); }
  IPAddress  ip()    const { return WiFi.status() == WL_CONNECTED ? WiFi.localIP() : WiFi.softAPIP(); }

private:
  bool waitForSTA_(uint32_t ms);

  struct TxCtx {
    WebWS*          ws = nullptr;
    PoseBus*        pose = nullptr;
    LidarService*   lidarSvc = nullptr;
    BoardPowerLib*  boardPower = nullptr;
    uint32_t        pose_period_ms  = 100;
    uint32_t        lidar_period_ms = 200;
  };
  static void txTaskThunk_(void* arg);

  bool sta_enabled_ = true;
  bool ap_enabled_  = true;
  bool wifi_sleep_  = false;
  uint32_t sta_timeout_ms_ = 15000;
  String sta_ssid_, sta_pass_;
  String ap_ssid_ = "BugBot", ap_pass_ = "changeme123";
  String hostname_;
  wifi_power_t tx_power_ = WIFI_POWER_15dBm;
  bool started_    = false;

  TaskHandle_t tx_task_ = nullptr;
  TxCtx tx_;
};