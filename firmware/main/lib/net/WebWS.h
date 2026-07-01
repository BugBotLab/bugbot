// WebWS.h
// AsyncWebServer + WebSocket facade (port 80, endpoint /ws). Owns LittleFS static-file
// serving, all inbound control callbacks, and all binary telemetry send helpers.
// Binary packet formats are documented in the send*() method signatures below.
#pragma once
#include <Arduino.h>
#include <functional>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <freertos/semphr.h>

class WebWS {
public:
  // Start only the HTTP/WebSocket server on the already-active network
  bool beginServer();
  AsyncWebServer& server() { return server_; }

  // Legacy entry points kept for compatibility.
  // They now ONLY start the server and do not touch Wi-Fi state.
  bool beginAP(const char* ssid, const char* pass);
  bool beginSTA(const char* ssid, const char* pass);

  // Optional: serve /index.html + assets from LittleFS.
  // IMPORTANT: call this BEFORE beginServer()/beginAP()/beginSTA()
  void serveFilesFromFS(bool enable = true);

  // Legacy control callback: discrete direction + speed
  void onControl(std::function<void(uint8_t, float)> cb) { ctrl_cb_ = std::move(cb); }

  // Vector control callback: longitudinal/log/long, lateral/lat, rotational/rot in [-1, +1]
  void onMotionVec(std::function<void(float, float, float)> cb) { vec_cb_ = std::move(cb); }

  // Raw text callback for actuator/UI JSON messages
  void onText(std::function<void(const char*)> cb) { text_cb_ = std::move(cb); }

  // Fired when the first client connects (0→1 transition)
  void onClientConnect(std::function<void()> cb)    { conn_cb_ = std::move(cb); }
  // Fired when the last client disconnects (1→0 transition)
  void onClientDisconnect(std::function<void()> cb) { disc_cb_ = std::move(cb); }

  // Outbound (ESP32 -> PC)
  void sendPose(uint32_t seq, uint32_t t_ms, float x_mm, float y_mm, float yaw_deg);

  // LiDAR frames (Header A: <I I B B H>) — kept for backwards compatibility
  void sendLidar4(uint32_t seq, uint32_t t_ms, uint8_t row, uint8_t vmask, uint16_t hz,
                  const float xw[4], const float yw[4]);
  void sendLidar8(uint32_t seq, uint32_t t_ms, uint8_t row, uint8_t vmask, uint16_t hz,
                  const float xw[8], const float yw[8]);

  // 3-D scan packet: magic 0x5343, then n world-frame points (x, y, z in mm).
  // Packet layout: [u16 magic][u32 seq][u32 t_ms][u8 n] + n×[f32 x, f32 y, f32 z]
  // Max n = 16 (full 4×4 grid).
  void sendScan3D(uint32_t seq, uint32_t t_ms, uint8_t n,
                  const float xw[], const float yw[], const float zw[]);

  // AprilTag detections (robot → host)
  // Packet: [uint16 magic=0x4154][uint32 seq][uint32 t_ms][uint8 count]
  //         [per hit: uint8 id, float cx_px, cy_px, az_deg, el_deg, dist_mm]
  struct AprilTagHit {
    uint8_t id;
    float cx_px, cy_px;   // tag centre in image pixels (QQVGA: 0–159, 0–119)
    float az_deg;         // positive = right of camera centre
    float el_deg;         // positive = above camera centre
    float dist_mm;        // 3-D range from homography translation vector |t|
    float tag_yaw_deg;    // tag face rotation around camera vertical axis, degrees
                          // 0 = tag faces camera straight-on; not sent over wire
  };
  void sendAprilTags(uint32_t seq, uint32_t t_ms,
                     uint8_t count, const AprilTagHit* hits);

  // Snapshot packet: magic 0x494D — JPEG image captured on AprilTag detection.
  // [u16 magic][u32 seq][u32 t_ms][u8 tag_id][u32 img_len][u8[] jpeg_data]
  void sendSnapshot(uint32_t seq, uint32_t t_ms,
                    uint8_t tag_id, const uint8_t* jpeg, size_t jpeg_len);

  // Occupancy-map packet: magic 0x4D41 — full grid
  // [u16 magic][u32 seq][u32 t_ms][f32 rx][f32 ry][f32 ryaw_rad]
  // [i32 origin_x_mm][i32 origin_y_mm][u8 cell_mm][u8 nx][u8 ny][u8 cells[ny*nx]]
  void sendMap(uint32_t seq, uint32_t t_ms,
               float robot_x_mm, float robot_y_mm, float robot_yaw_rad,
               int32_t origin_x_mm, int32_t origin_y_mm,
               uint8_t cell_mm, uint8_t nx, uint8_t ny,
               const uint8_t* cells);

  // Occupancy-map delta packet: magic 0x4D44 — changed cells only
  // Same 33-byte header as sendMap, then [u16 n_cells] + n×[u8 row, col, val]
  // triplets: packed array of [row, col, val] triples, length = n_cells * 3
  void sendMapDelta(uint32_t seq, uint32_t t_ms,
                    float robot_x_mm, float robot_y_mm, float robot_yaw_rad,
                    int32_t origin_x_mm, int32_t origin_y_mm,
                    uint8_t cell_mm, uint8_t nx, uint8_t ny,
                    const uint8_t* triplets, uint16_t n_cells);

  // Text telemetry / status
  void sendText(const char* s);

  // Stop WS/web server cleanly for low-power mode
  void stop();

  // Housekeeping; safe to call often (internally rate-limited)
  void poll();

  // Connection state for arbitration with local LED/status logic
  size_t connectedClients() const { return clientCount_; }
  uint32_t lastDisconnectMs() const { return lastDisconnectMs_; }

private:
  void installHandlers_();
  void startServer_();

  AsyncWebServer server_{80};
  AsyncWebSocket ws_{"/ws"};
  SemaphoreHandle_t sendMtx_ = nullptr;
  bool fsServed_ = false;
  bool fsMounted_ = false;
  bool staticInstalled_ = false;
  bool handlersInstalled_ = false;
  bool serverStarted_ = false;
  volatile size_t clientCount_ = 0;
  volatile uint32_t lastDisconnectMs_ = 0;

  std::function<void(uint8_t, float)> ctrl_cb_{};
  std::function<void(float, float, float)> vec_cb_{};
  std::function<void(const char*)> text_cb_{};
  std::function<void()> conn_cb_{};
  std::function<void()> disc_cb_{};
};
