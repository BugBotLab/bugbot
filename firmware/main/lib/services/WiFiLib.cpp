#include "WiFiLib.h"
#include <freertos/idf_additions.h>

// -------------------- Public --------------------

void WiFiLib::start(PoseGetter poseGetter, int coreWiFi, int coreNet, int udpHz) {
  _getPose = poseGetter;
  _udpHz = (udpHz <= 0) ? 1 : udpHz;   // clamp
  _coreNet = coreNet;

  // One-shot WiFi bring-up (only this task for now)
  xTaskCreatePinnedToCoreWithCaps(WiFiLib::TaskWiFiThunk, "wifi", 8192, this, 3, nullptr, coreWiFi,
                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

size_t WiFiLib::peerCount() const {
  size_t n = 0;
  portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_peersMux));
  for (const auto& p : _peers) if (p.valid) ++n;
  portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_peersMux));
  return n;
}

// -------------------- Task Thunks --------------------

void WiFiLib::TaskWiFiThunk(void* arg)    { static_cast<WiFiLib*>(arg)->TaskWiFi();    vTaskDelete(nullptr); }
void WiFiLib::TaskUDP_RX_Thunk(void* arg) { static_cast<WiFiLib*>(arg)->TaskUDP_RX(); }
void WiFiLib::TaskUDP_TX_Thunk(void* arg) { static_cast<WiFiLib*>(arg)->TaskUDP_TX(); }

// -------------------- Private: WiFi bring-up --------------------

void WiFiLib::TaskWiFi() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(BugBotNet::WIFI_SLEEP);

  // AP
  if (BugBotNet::USE_AP) {
    if (BugBotNet::AP_USE_STATIC_IP) {
      WiFi.softAPConfig(BugBotNet::AP_IP, BugBotNet::AP_GATEWAY, BugBotNet::AP_SUBNET);
    }
    bool apok = WiFi.softAP(BugBotNet::AP_SSID, BugBotNet::AP_PASS);
    Serial.printf("AP: %s  IP=%s\n", apok ? "started" : "FAILED",
                  WiFi.softAPIP().toString().c_str());
  }

  // STA
  if (BugBotNet::USE_STA) {
    if (BugBotNet::USE_STATIC_IP) {
      WiFi.config(BugBotNet::STATIC_IP, BugBotNet::GATEWAY, BugBotNet::SUBNET, BugBotNet::DNS1);
    }
    WiFi.begin(BugBotNet::WIFI_SSID, BugBotNet::WIFI_PASS);
    Serial.printf("STA: connecting to %s", BugBotNet::WIFI_SSID);
    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries++ < 60) {
      Serial.print(".");
      vTaskDelay(pdMS_TO_TICKS(500));
    }
    Serial.printf("\nSTA: %s  IP=%s\n",
      (WiFi.status()==WL_CONNECTED) ? "connected" : "NOT connected",
      WiFi.localIP().toString().c_str());
  }

  // mDNS (optional)
  if (BugBotNet::USE_MDNS) {
    if (MDNS.begin(BugBotNet::MDNS_NAME)) {
      Serial.printf("mDNS: http://%s.local\n", BugBotNet::MDNS_NAME);
    } else {
      Serial.println("mDNS: failed");
    }
  }

  // Mark Wi-Fi as ready and THEN create UDP tasks on the network core
  _wifiReady = true;
  xTaskCreatePinnedToCoreWithCaps(WiFiLib::TaskUDP_RX_Thunk, "udp_rx", 8192, this, 2, nullptr, _coreNet,
                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  xTaskCreatePinnedToCoreWithCaps(WiFiLib::TaskUDP_TX_Thunk, "udp_tx", 8192, this, 2, nullptr, _coreNet,
                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

// -------------------- Private: UDP RX/TX --------------------

void WiFiLib::TaskUDP_RX() {
  // Wait for Wi-Fi bring-up to finish (avoid NULL queue races)
  while (!_wifiReady) vTaskDelay(pdMS_TO_TICKS(50));
  vTaskDelay(pdMS_TO_TICKS(100)); // tiny extra guard

  _udpRX.begin(BugBotNet::REG_PORT);
  Serial.printf("UDP RX (register) listening on %u\n", BugBotNet::REG_PORT);

  uint8_t buf[64];
  for (;;) {
    int n = _udpRX.parsePacket();
    if (n > 0) {
      IPAddress ip = _udpRX.remoteIP();
      upsertPeer(ip);
      _udpRX.read(buf, min(n, (int)sizeof(buf))); // drain
    }
    prunePeers();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void WiFiLib::TaskUDP_TX() {
  // Wait for Wi-Fi bring-up to finish (avoid NULL queue races)
  while (!_wifiReady) vTaskDelay(pdMS_TO_TICKS(50));
  vTaskDelay(pdMS_TO_TICKS(100)); // tiny extra guard

  // Use ephemeral local port for TX (needs an argument on core 3.x)
  _udpTX.begin(0);

  const TickType_t period = pdMS_TO_TICKS(1000 / _udpHz);
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    // ---- Pose ----
    if (_getPose) {
      float x=0, y=0, yawDeg=0;
      _getPose(x, y, yawDeg);
      PosePkt pkt { _seq++, (uint32_t)millis(), x, y, yawDeg };
      send_to_all(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
    }

    // ---- Lidar world points (if provided) ----
    if (_getLidar) {
      float xw[4], yw[4];
      uint8_t row = 0, vmask = 0;
      if (_getLidar(xw, yw, vmask, row)) {
        LidarPkt lp{};
        lp.seq        = _seq++;
        lp.t_ms       = (uint32_t)millis();
        lp.row        = row;
        lp.hz         = 0;                 // leave 0 if unknown
        lp.valid_mask = vmask;
        lp.reserved   = 0;
        for (int i=0;i<4;i++) { lp.xw[i] = xw[i]; lp.yw[i] = yw[i]; }
        send_to_all(reinterpret_cast<const uint8_t*>(&lp), sizeof(lp));
      }
    }

    vTaskDelayUntil(&last, period);
  }
}

// -------------------- Private: peer registry + send helper --------------------

void WiFiLib::upsertPeer(const IPAddress& ip) {
  const uint32_t now = millis();
  portENTER_CRITICAL(&_peersMux);
  // Update if exists
  for (auto& p : _peers) {
    if (p.valid && p.ip == ip) { p.last_ms = now; portEXIT_CRITICAL(&_peersMux); return; }
  }
  // Find free/expired
  for (auto& p : _peers) {
    if (!p.valid || (now - p.last_ms > BugBotNet::CLIENT_TTL_MS)) {
      p.ip = ip; p.last_ms = now; p.valid = true; portEXIT_CRITICAL(&_peersMux); return;
    }
  }
  // Overwrite oldest
  size_t idx = 0; uint32_t oldestAge = 0;
  for (size_t i = 0; i < MAX_PEERS; ++i) {
    uint32_t age = now - _peers[i].last_ms;
    if (age > oldestAge) { oldestAge = age; idx = i; }
  }
  _peers[idx] = { ip, now, true };
  portEXIT_CRITICAL(&_peersMux);
}

void WiFiLib::prunePeers() {
  const uint32_t now = millis();
  portENTER_CRITICAL(&_peersMux);
  for (auto& p : _peers) {
    if (p.valid && (now - p.last_ms > BugBotNet::CLIENT_TTL_MS)) p.valid = false;
  }
  portEXIT_CRITICAL(&_peersMux);
}

void WiFiLib::send_to_all(const uint8_t* data, size_t len) {
  // 1) Fixed STA target
  if (BugBotNet::USE_STA && WiFi.status() == WL_CONNECTED) {
    _udpTX.beginPacket(BugBotNet::REMOTE_IP, BugBotNet::UDP_PORT);
    _udpTX.write(data, len);
    _udpTX.endPacket();
  }

  // 2) Registered peers (AP or STA)
  portENTER_CRITICAL(&_peersMux);
  for (const auto& p : _peers) {
    if (!p.valid) continue;
    _udpTX.beginPacket(p.ip, BugBotNet::UDP_PORT);
    _udpTX.write(data, len);
    _udpTX.endPacket();
  }
  portEXIT_CRITICAL(&_peersMux);

  // 3) Optional AP broadcast
  if (BugBotNet::USE_AP && BugBotNet::AP_BROADCAST) {
    IPAddress bcast = WiFi.softAPIP();
    bcast[3] = 255; // assumes /24
    _udpTX.beginPacket(bcast, BugBotNet::UDP_PORT);
    _udpTX.write(data, len);
    _udpTX.endPacket();
  }
}
