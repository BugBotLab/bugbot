#include "WebWS.h"
#include <stdlib.h>
#include <string.h>

namespace {
static bool parseNamedFloat_(const char* msg, const char* key, float& out) {
  if (!msg || !key) return false;
  const char* p = strstr(msg, key);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  ++p;
  char* end = nullptr;
  out = strtof(p, &end);
  return end != p;
}

static bool parseMotionVecJson_(const char* msg, float& longitudinal, float& lateral, float& rotational) {
  float a = 0.0f, b = 0.0f, c = 0.0f;
  bool okA = parseNamedFloat_(msg, "longitudinal", a) || parseNamedFloat_(msg, "long", a) || parseNamedFloat_(msg, "log", a) || parseNamedFloat_(msg, "vx", a);
  bool okB = parseNamedFloat_(msg, "lateral", b) || parseNamedFloat_(msg, "lat", b) || parseNamedFloat_(msg, "vy", b);
  bool okC = parseNamedFloat_(msg, "rotational", c) || parseNamedFloat_(msg, "rot", c) || parseNamedFloat_(msg, "omega", c);
  if (!(okA || okB || okC)) return false;
  longitudinal = a;
  lateral = b;
  rotational = c;
  return true;
}
}

bool WebWS::beginServer() {
  if (serverStarted_) return true;
  startServer_();
  serverStarted_ = true;
  return true;
}

bool WebWS::beginAP(const char* ssid, const char* pass) {
  (void)ssid;
  (void)pass;
  return beginServer();
}

bool WebWS::beginSTA(const char* ssid, const char* pass) {
  (void)ssid;
  (void)pass;
  return beginServer();
}

void WebWS::serveFilesFromFS(bool enable) {
  fsServed_ = enable;
}

void WebWS::installHandlers_() {
  if (handlersInstalled_) return;

  ws_.onEvent([this](AsyncWebSocket* s,
                     AsyncWebSocketClient* c,
                     AwsEventType type,
                     void* arg,
                     uint8_t* data,
                     size_t len) {
    (void)s;

    switch (type) {
      case WS_EVT_CONNECT:
        ws_.cleanupClients();   // remove any zombie (disconnected) clients; no hard limit enforced
        ++clientCount_;
        Serial.printf("[WS] CONNECT id=%lu ip=%s t=%lu clients=%u\n",
                      (unsigned long)c->id(), c->remoteIP().toString().c_str(), millis(), (unsigned)clientCount_);
        if (clientCount_ == 1 && conn_cb_) conn_cb_();   // first client
        break;

      case WS_EVT_DISCONNECT:
        if (clientCount_ > 0) --clientCount_;
        if (clientCount_ == 0) {
          lastDisconnectMs_ = millis();
          if (disc_cb_) disc_cb_();                       // last client left
        }
        Serial.printf("[WS] DISCONNECT id=%lu t=%lu clients=%u\n", (unsigned long)c->id(), millis(), (unsigned)clientCount_);
        break;

      case WS_EVT_ERROR:
        Serial.printf("[WS] ERROR id=%lu t=%lu\n", (unsigned long)(c ? c->id() : 0), millis());
        break;

      case WS_EVT_PING:
      case WS_EVT_PONG:
        break;

      case WS_EVT_DATA: {
        if (!c || !data || len == 0) break;

        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (!info->final || info->index != 0) break;

        if (info->opcode == WS_BINARY) {
          // Legacy packet: <uint16 magic=0x4242, uint32 seq, uint8 dir, float speed>
          if (len == (2 + 4 + 1 + 4)) {
            uint16_t magic;
            uint32_t seq;
            uint8_t dir;
            float speed;

            memcpy(&magic, data + 0, 2);
            memcpy(&seq,   data + 2, 4);
            memcpy(&dir,   data + 6, 1);
            memcpy(&speed, data + 7, 4);

            if (magic == 0x4242) {
              (void)seq;
              if (ctrl_cb_) ctrl_cb_(dir, speed);
            }
          }
          // New vector packet: <uint16 magic=0x5658, uint32 seq, float longitudinal, float lateral, float rotational>
          else if (len == (2 + 4 + 4 + 4 + 4)) {
            uint16_t magic;
            uint32_t seq;
            float longitudinal, lateral, rotational;

            memcpy(&magic,        data + 0,  2);
            memcpy(&seq,          data + 2,  4);
            memcpy(&longitudinal, data + 6,  4);
            memcpy(&lateral,      data + 10, 4);
            memcpy(&rotational,   data + 14, 4);

            if (magic == 0x5658 || magic == 0x7678) {
              (void)seq;
              if (vec_cb_) vec_cb_(longitudinal, lateral, rotational);
            }
          }
        } else if (info->opcode == WS_TEXT) {
          char* msg = (char*)malloc(len + 1);
          if (!msg) break;

          memcpy(msg, data, len);
          msg[len] = '\0';

          float longitudinal = 0.0f, lateral = 0.0f, rotational = 0.0f;
          const bool isVec = parseMotionVecJson_(msg, longitudinal, lateral, rotational);
          if (isVec) {
            if (vec_cb_) vec_cb_(longitudinal, lateral, rotational);
          } else {
            if (text_cb_) text_cb_(msg);
          }

          free(msg);
        }
        break;
      }
    }
  });

  server_.addHandler(&ws_);

  server_.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->redirect("/config");
  });

  handlersInstalled_ = true;
}

static inline void wsSend_(AsyncWebSocket& ws, SemaphoreHandle_t mtx,
                            const uint8_t* buf, size_t len) {
  if (xSemaphoreTake(mtx, pdMS_TO_TICKS(20)) == pdTRUE) {
    ws.binaryAll(buf, len);
    xSemaphoreGive(mtx);
  }
}

void WebWS::startServer_() {
  if (!sendMtx_) sendMtx_ = xSemaphoreCreateMutex();
  if (fsServed_) {
    if (!fsMounted_) {
      fsMounted_ = LittleFS.begin(true, "/littlefs", 10, "spiffs");
    }
    if (fsMounted_ && !staticInstalled_) {
      server_.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
      staticInstalled_ = true;
    }
  }

  installHandlers_();
  server_.begin();
}

void WebWS::stop() {
  if (!serverStarted_) return;

  Serial.println("[WebWS] stop(): closing ws clients");
  ws_.closeAll();
  ws_.cleanupClients();
  clientCount_ = 0;
  lastDisconnectMs_ = millis();
  delay(100);

  Serial.println("[WebWS] stop(): ending server");
  server_.end();
  delay(100);

  serverStarted_ = false;
}

void WebWS::poll() {
  static uint32_t lastClean = 0;
  const uint32_t now = millis();
  if (now - lastClean >= 5000) {
    ws_.cleanupClients();
    lastClean = now;
  }
}

void WebWS::sendPose(uint32_t seq, uint32_t t_ms, float x_mm, float y_mm, float yaw_deg) {
  if (ws_.count() == 0) return;
  uint8_t buf[20];
  memcpy(buf + 0,  &seq,     4);
  memcpy(buf + 4,  &t_ms,    4);
  memcpy(buf + 8,  &x_mm,    4);
  memcpy(buf + 12, &y_mm,    4);
  memcpy(buf + 16, &yaw_deg, 4);
  wsSend_(ws_, sendMtx_, buf, sizeof(buf));
}

void WebWS::sendLidar4(uint32_t seq, uint32_t t_ms, uint8_t row, uint8_t vmask, uint16_t hz,
                       const float xw[4], const float yw[4]) {
  if (ws_.count() == 0) return;
  uint8_t buf[44];
  memcpy(buf + 0,  &seq,   4);
  memcpy(buf + 4,  &t_ms,  4);
  memcpy(buf + 8,  &row,   1);
  memcpy(buf + 9,  &vmask, 1);
  memcpy(buf + 10, &hz,    2);
  memcpy(buf + 12, xw,     4 * 4);
  memcpy(buf + 28, yw,     4 * 4);
  wsSend_(ws_, sendMtx_, buf, sizeof(buf));
}

void WebWS::sendLidar8(uint32_t seq, uint32_t t_ms, uint8_t row, uint8_t vmask, uint16_t hz,
                       const float xw[8], const float yw[8]) {
  if (ws_.count() == 0) return;
  uint8_t buf[76];
  memcpy(buf + 0,  &seq,   4);
  memcpy(buf + 4,  &t_ms,  4);
  memcpy(buf + 8,  &row,   1);
  memcpy(buf + 9,  &vmask, 1);
  memcpy(buf + 10, &hz,    2);
  memcpy(buf + 12, xw,     8 * 4);
  memcpy(buf + 44, yw,     8 * 4);
  wsSend_(ws_, sendMtx_, buf, sizeof(buf));
}

void WebWS::sendScan3D(uint32_t seq, uint32_t t_ms, uint8_t n,
                       const float xw[], const float yw[], const float zw[]) {
  if (ws_.count() == 0 || n == 0) return;
  const uint8_t np = (n > 32) ? 32 : n;
  // header: magic(2) + seq(4) + t_ms(4) + n(1) = 11 bytes
  // data:   np * (x+y+z floats) = np * 12 bytes
  const size_t sz = 11 + (size_t)np * 12;
  uint8_t buf[11 + 32 * 12];
  const uint16_t magic = 0x5343;
  memcpy(buf + 0, &magic, 2);
  memcpy(buf + 2, &seq,   4);
  memcpy(buf + 6, &t_ms,  4);
  buf[10] = np;
  for (uint8_t i = 0; i < np; i++) {
    uint8_t* p = buf + 11 + i * 12;
    memcpy(p + 0, &xw[i], 4);
    memcpy(p + 4, &yw[i], 4);
    memcpy(p + 8, &zw[i], 4);
  }
  wsSend_(ws_, sendMtx_, buf, sz);
}

void WebWS::sendSnapshot(uint32_t seq, uint32_t t_ms,
                         uint8_t tag_id, const uint8_t* jpeg, size_t jpeg_len) {
  if (ws_.count() == 0 || !jpeg || jpeg_len == 0) return;
  const size_t sz = 15 + jpeg_len;
  uint8_t* buf = (uint8_t*)malloc(sz);
  if (!buf) return;
  const uint16_t magic = 0x494D;
  uint32_t img_len32 = (uint32_t)jpeg_len;
  memcpy(buf + 0,  &magic,    2);
  memcpy(buf + 2,  &seq,      4);
  memcpy(buf + 6,  &t_ms,     4);
  buf[10] = tag_id;
  memcpy(buf + 11, &img_len32, 4);
  memcpy(buf + 15, jpeg, jpeg_len);
  wsSend_(ws_, sendMtx_, buf, sz);
  free(buf);
}

void WebWS::sendMap(uint32_t seq, uint32_t t_ms,
                    float robot_x_mm, float robot_y_mm, float robot_yaw_rad,
                    int32_t origin_x_mm, int32_t origin_y_mm,
                    uint8_t cell_mm, uint8_t nx, uint8_t ny,
                    const uint8_t* cells) {
  if (ws_.count() == 0) return;
  const size_t cell_bytes = (size_t)nx * ny;
  const size_t sz = 33 + cell_bytes;
  uint8_t* buf = (uint8_t*)malloc(sz);
  if (!buf) return;
  const uint16_t magic = 0x4D41;
  memcpy(buf + 0,  &magic,           2);
  memcpy(buf + 2,  &seq,             4);
  memcpy(buf + 6,  &t_ms,            4);
  memcpy(buf + 10, &robot_x_mm,      4);
  memcpy(buf + 14, &robot_y_mm,      4);
  memcpy(buf + 18, &robot_yaw_rad,   4);
  memcpy(buf + 22, &origin_x_mm,     4);
  memcpy(buf + 26, &origin_y_mm,     4);
  buf[30] = cell_mm;
  buf[31] = nx;
  buf[32] = ny;
  memcpy(buf + 33, cells, cell_bytes);
  wsSend_(ws_, sendMtx_, buf, sz);
  free(buf);
}

void WebWS::sendMapDelta(uint32_t seq, uint32_t t_ms,
                         float robot_x_mm, float robot_y_mm, float robot_yaw_rad,
                         int32_t origin_x_mm, int32_t origin_y_mm,
                         uint8_t cell_mm, uint8_t nx, uint8_t ny,
                         const uint8_t* triplets, uint16_t n_cells) {
  if (ws_.count() == 0 || n_cells == 0 || !triplets) return;
  const size_t sz = 35 + (size_t)n_cells * 3;
  uint8_t* buf = (uint8_t*)malloc(sz);
  if (!buf) return;
  const uint16_t magic = 0x4D44;
  memcpy(buf + 0,  &magic,         2);
  memcpy(buf + 2,  &seq,           4);
  memcpy(buf + 6,  &t_ms,          4);
  memcpy(buf + 10, &robot_x_mm,    4);
  memcpy(buf + 14, &robot_y_mm,    4);
  memcpy(buf + 18, &robot_yaw_rad, 4);
  memcpy(buf + 22, &origin_x_mm,   4);
  memcpy(buf + 26, &origin_y_mm,   4);
  buf[30] = cell_mm;
  buf[31] = nx;
  buf[32] = ny;
  memcpy(buf + 33, &n_cells, 2);
  memcpy(buf + 35, triplets, (size_t)n_cells * 3);
  wsSend_(ws_, sendMtx_, buf, sz);
  free(buf);
}

void WebWS::sendAprilTags(uint32_t seq, uint32_t t_ms,
                          uint8_t count, const AprilTagHit* hits) {
  if (ws_.count() == 0 || !hits || count == 0) return;
  const uint8_t n = (count > 4) ? 4 : count;
  // header 11 B + 21 B per hit, max 95 B total
  uint8_t buf[95];
  const uint16_t magic = 0x4154;
  memcpy(buf + 0,  &magic, 2);
  memcpy(buf + 2,  &seq,   4);
  memcpy(buf + 6,  &t_ms,  4);
  memcpy(buf + 10, &n,     1);
  for (uint8_t i = 0; i < n; i++) {
    uint8_t* p = buf + 11 + i * 21;
    memcpy(p + 0,  &hits[i].id,      1);
    memcpy(p + 1,  &hits[i].cx_px,   4);
    memcpy(p + 5,  &hits[i].cy_px,   4);
    memcpy(p + 9,  &hits[i].az_deg,  4);
    memcpy(p + 13, &hits[i].el_deg,  4);
    memcpy(p + 17, &hits[i].dist_mm, 4);
  }
  wsSend_(ws_, sendMtx_, buf, (size_t)(11 + n * 21));
}

void WebWS::sendText(const char* s) {
  if (ws_.count() == 0 || !s) return;
  ws_.textAll(s);
}
