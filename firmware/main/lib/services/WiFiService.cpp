#include "WiFiService.h"
#include "../net/WebWS.h"
#include "../core/PoseBus.h"
#include "../services/LidarService.h"
#include "../services/MotionService.h"
#include "../drivers/BoardPowerLib.h"
#include "../core/Transforms.hpp"
#include <esp_wifi.h>
#include <freertos/idf_additions.h>

extern volatile bool g_sleepEntryInProgress;

void WiFiService::configure(bool sta_enabled, bool ap_enabled,
                            const char* sta_ssid, const char* sta_pass,
                            bool wifi_sleep,
                            const char* ap_ssid, const char* ap_pass,
                            uint32_t sta_timeout_ms) {
  sta_enabled_ = sta_enabled;
  ap_enabled_ = ap_enabled;
  wifi_sleep_ = wifi_sleep;
  sta_timeout_ms_ = sta_timeout_ms;
  if (sta_ssid) sta_ssid_ = sta_ssid;
  if (sta_pass) sta_pass_ = sta_pass;
  if (ap_ssid)  ap_ssid_  = ap_ssid;
  if (ap_pass)  ap_pass_  = ap_pass;
}

void WiFiService::setHostname(const char* hn) {
  if (hn) hostname_ = hn;
}

void WiFiService::setTxPower(wifi_power_t pwr) {
  tx_power_ = pwr;
}

bool WiFiService::waitForSTA_(uint32_t ms) {
  const uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < ms) {
    delay(50);
  }
  return WiFi.status() == WL_CONNECTED;
}

bool WiFiService::start(WebWS& ws) {
  stop();
  delay(100);

  if (!sta_enabled_ && !ap_enabled_) {
    Serial.println("[WiFi] Both STA and AP disabled");
    started_ = false;
    return false;
  }

  if (wifi_sleep_) {
    WiFi.setSleep(true);
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
  } else {
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
  }

  if (hostname_.length()) {
    WiFi.setHostname(hostname_.c_str());
  }

  bool staUp = false;
  bool apUp = false;

  if (sta_enabled_) {
    if (sta_ssid_.isEmpty()) {
      Serial.println("[WiFi] STA skipped: SSID is blank");
    } else {
      Serial.printf("[WiFi] STA: SSID='%s'\n", sta_ssid_.c_str());
      WiFi.mode(ap_enabled_ ? WIFI_AP_STA : WIFI_STA);
      delay(50);
      WiFi.begin(sta_ssid_.c_str(), sta_pass_.c_str());
      staUp = waitForSTA_(sta_timeout_ms_);
      if (staUp) {
        Serial.printf("[WiFi] STA up: %s RSSI=%d dBm\n",
                      WiFi.localIP().toString().c_str(), WiFi.RSSI());
      } else {
        Serial.println("[WiFi] STA connect FAILED");
      }
    }
  }

  if (ap_enabled_ && (!staUp || ap_enabled_)) {
    if (!sta_enabled_) {
      WiFi.mode(WIFI_AP);
      delay(50);
    }

    Serial.printf("[WiFi] AP: SSID='%s'\n", ap_ssid_.c_str());
    apUp = WiFi.softAP(ap_ssid_.c_str(), ap_pass_.c_str());
    if (apUp) {
      Serial.printf("[WiFi] AP up: %s\n", WiFi.softAPIP().toString().c_str());
    } else {
      Serial.println("[WiFi] AP start FAILED");
    }
  }

  if (!staUp && !apUp) {
    started_ = false;
    return false;
  }

  WiFi.setTxPower(tx_power_);
  started_ = ws.beginServer();
  return started_;
}

bool WiFiService::startWithTelemetry(WebWS& ws,
                                     PoseBus& pose,
                                     LidarService& lidarSvc,
                                     MotionService& motionSvc,
                                     BoardPowerLib& boardPower,
                                     uint32_t pose_hz,
                                     uint32_t lidar_hz,
                                     uint32_t tx_stack_kb,
                                     UBaseType_t tx_prio,
                                     BaseType_t tx_core) {
  if (!start(ws)) return false;

  ws.onControl([&motionSvc](uint8_t dir, float speed) {
    if (g_sleepEntryInProgress) return;
    motionSvc.setCommand(static_cast<DriveDir>(dir), speed);
  });

  ws.onMotionVec([&motionSvc](float longitudinal, float lateral, float rotational) {
    if (g_sleepEntryInProgress) return;
    motionSvc.setCommandVec(longitudinal, lateral, rotational);
  });

  tx_.ws = &ws;
  tx_.pose = &pose;
  tx_.lidarSvc = &lidarSvc;
  tx_.boardPower = &boardPower;
  tx_.pose_period_ms  = (pose_hz  > 0) ? (1000u / pose_hz)  : 1000u;
  tx_.lidar_period_ms = (lidar_hz > 0) ? (1000u / lidar_hz) : 1000u;

  const uint32_t stack_words = (tx_stack_kb * 1024u) / sizeof(StackType_t);
  BaseType_t ok = xTaskCreatePinnedToCoreWithCaps(
      txTaskThunk_,
      "net_tx",
      stack_words,
      &tx_,
      tx_prio,
      &tx_task_,
      tx_core,
      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  if (ok != pdPASS) {
    Serial.println("[WiFi] TX task create FAILED");
    tx_task_ = nullptr;
    return false;
  }

  Serial.printf("[WiFi] TX task started (pose=%lu Hz, lidar=%lu Hz)\n",
                (unsigned long)pose_hz,
                (unsigned long)lidar_hz);
  return true;
}

void WiFiService::stop() {
  if (tx_task_) {
    vTaskDelete(tx_task_);
    tx_task_ = nullptr;
    Serial.println("[WiFi] tx task deleted");
  }

  started_ = false;

  Serial.printf("[WiFi] stop(): before mode=%d status=%d\n",
                (int)WiFi.getMode(), (int)WiFi.status());

  WiFi.disconnect(true, true);
  delay(100);

  WiFi.softAPdisconnect(true);
  delay(100);

  esp_wifi_disconnect();
  delay(100);

  esp_wifi_stop();
  delay(100);

  WiFi.mode(WIFI_OFF);
  delay(100);

  Serial.printf("[WiFi] stop(): after mode=%d status=%d\n",
                (int)WiFi.getMode(), (int)WiFi.status());
}

void WiFiService::txTaskThunk_(void* arg) {
  TxCtx* ctx = reinterpret_cast<TxCtx*>(arg);
  WebWS& ws = *ctx->ws;
  PoseBus& pose = *ctx->pose;
  BoardPowerLib* boardPower = ctx->boardPower;

  uint32_t seq_pose  = 0;

  const uint32_t T_POSE_MS = ctx->pose_period_ms;
  const uint32_t T_BAT_MS  = 10000;

  uint32_t t_pose_next = millis() + T_POSE_MS;
  uint32_t t_bat_next  = millis() + T_BAT_MS;
  uint32_t t_wifi_next  = millis() + 30000;  // reconnect check every 30 s

  for (;;) {
    const uint32_t now = millis();

    // STA reconnect — only when STA mode is active; AP-only mode must not reconnect
    if ((int32_t)(now - t_wifi_next) >= 0) {
      t_wifi_next += 30000;
      const wifi_mode_t wm = WiFi.getMode();
      if ((wm == WIFI_STA || wm == WIFI_AP_STA) && WiFi.status() != WL_CONNECTED) {
        Serial.printf("[WiFi] TX: STA link lost (status=%d), reconnecting…\n", (int)WiFi.status());
        WiFi.reconnect();
      }
    }

    if ((int32_t)(now - t_pose_next) >= 0) {
      const auto P = pose.get();
      ws.sendPose(seq_pose++, now, P.x_mm, P.y_mm, rad2deg(P.yaw_rad));
      t_pose_next += T_POSE_MS;
    }

    if ((int32_t)(now - t_bat_next) >= 0) {
      if (boardPower) {
        const float bat = boardPower->readBatteryVolts();
        char msg[64];
        snprintf(msg, sizeof(msg), "{\"battery_volts\":%.2f}", bat);
        ws.sendText(msg);
      }
      t_bat_next += T_BAT_MS;
    }

    ws.poll();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
