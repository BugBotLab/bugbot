// WiFiLib.h
// Legacy UDP-based telemetry sender (PosePkt, LidarPkt over raw UDP).
// Superseded by WiFiService + WebWS for new features; retained as an alternative
// transport path. Uses compile-time credentials from WiFiConfig.h / BugBotNet::.
#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include "../config/WiFiConfig.h"   // expects BugBotNet::* config (SSID, ports, IPs, flags)

// Pose packet
#pragma pack(push,1)
struct PosePkt { uint32_t seq, t_ms; float x_mm, y_mm, yaw_deg; };
#pragma pack(pop)

// Lidar packet: 4 world points, bitmask validity
#pragma pack(push,1)
struct LidarPkt {
  uint32_t seq, t_ms;
  uint8_t  row;          // 0..3 strip row
  uint8_t  hz;           // publish rate (informational)
  uint8_t  valid_mask;   // lower 4 bits used
  uint8_t  reserved;     // pad
  float    xw[4];        // mm
  float    yw[4];        // mm
};
#pragma pack(pop)

// Sketch-provided callbacks
typedef void (*PoseGetter)(float& x_mm, float& y_mm, float& yaw_deg);
typedef bool (*LidarGetter)(float xw[4], float yw[4], uint8_t &valid_mask, uint8_t &row);

class WiFiLib {
public:
  WiFiLib() = default;

  // Starts Wi-Fi bring-up now; UDP RX/TX tasks will start AFTER Wi-Fi init
  // poseGetter: callback to fetch latest pose
  // coreWiFi:   core for Wi-Fi bring-up task (default 0)
  // coreNet:    core for RX/TX loops (default 1)
  // udpHz:      TX rate in Hz (default 100)
  void start(PoseGetter poseGetter, int coreWiFi = 0, int coreNet = 1, int udpHz = 100);

  // Optional: also send LIDAR world points
  void setLidarGetter(LidarGetter g) { _getLidar = g; }

  // Helpers for your console print
  size_t peerCount() const;
  bool staConnected() const { return WiFi.status() == WL_CONNECTED; }
  bool apActive()     const { return (WiFi.getMode() & WIFI_AP);   }

private:
  // Task thunks
  static void TaskWiFiThunk(void* arg);
  static void TaskUDP_RX_Thunk(void* arg);
  static void TaskUDP_TX_Thunk(void* arg);

  // Real task bodies
  void TaskWiFi();          // bring-up AP/STA + optional mDNS; then spawn UDP tasks
  void TaskUDP_RX();        // waits until Wi-Fi ready, then begin() and loop
  void TaskUDP_TX();        // waits until Wi-Fi ready, then begin() and loop

  // Peer registry
  struct UdpPeer { IPAddress ip; uint32_t last_ms; bool valid; };
  static constexpr size_t MAX_PEERS = 8;
  void upsertPeer(const IPAddress& ip);
  void prunePeers();

  // Helper to send a blob to all destinations
  void send_to_all(const uint8_t* data, size_t len);

private:
  PoseGetter  _getPose  = nullptr;
  LidarGetter _getLidar = nullptr;

  WiFiUDP    _udpTX, _udpRX;
  volatile uint32_t _seq = 0;

  // runtime flags
  volatile bool _wifiReady = false;  // set true when Wi-Fi bring-up finished
  int _udpHz = 100;
  int _coreNet = 1;

  // Registry (protected by a spinlock-style mux)
  UdpPeer     _peers[MAX_PEERS] = {};
  mutable portMUX_TYPE _peersMux = portMUX_INITIALIZER_UNLOCKED;
};
