// lib/services/CameraAPI.cpp
#include "CameraAPI.h"
#include <esp_http_server.h>
#include <esp_camera.h>
#include <freertos/semphr.h>
#include <string.h>

static httpd_handle_t    s_srv    = nullptr;
static SemaphoreHandle_t s_camMtx = nullptr;

// True while MJPEG is streaming. AprilTagService checks this to pause frame capture
// so the stream task has exclusive use of the camera buffer.
volatile bool g_cameraStreaming = false;

// ── Helpers ───────────────────────────────────────────────────────────────────

static inline void set_cors_no_cache(httpd_req_t *req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin",  "*");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "*");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET,OPTIONS");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
  httpd_resp_set_hdr(req, "Pragma",   "no-cache");
  httpd_resp_set_hdr(req, "Expires",  "0");
}

static esp_err_t options_handler(httpd_req_t *req) {
  set_cors_no_cache(req);
  httpd_resp_set_status(req, "204 No Content");
  return httpd_resp_send(req, NULL, 0);
}

// Returns a JPEG buffer for fb regardless of pixel format.
// If already JPEG, points into fb->buf (must_free=false).
// Otherwise allocates via frame2jpg (must_free=true; caller must free()).
static bool get_jpeg_(camera_fb_t* fb,
                      uint8_t** jpg_buf, size_t* jpg_len, bool* must_free) {
  if (fb->format == PIXFORMAT_JPEG) {
    *jpg_buf = fb->buf; *jpg_len = fb->len; *must_free = false; return true;
  }
  *must_free = true;
  return frame2jpg(fb, 12, jpg_buf, jpg_len);
}

// ── Single-frame JPEG ─────────────────────────────────────────────────────────

static esp_err_t jpg_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return httpd_resp_send_500(req);
  uint8_t* jpg_buf; size_t jpg_len; bool must_free;
  if (!get_jpeg_(fb, &jpg_buf, &jpg_len, &must_free)) {
    esp_camera_fb_return(fb); return httpd_resp_send_500(req);
  }
  httpd_resp_set_type(req, "image/jpeg");
  set_cors_no_cache(req);
  esp_err_t rc = httpd_resp_send(req, (const char*)jpg_buf, jpg_len);
  if (must_free) free(jpg_buf);
  esp_camera_fb_return(fb);
  return rc;
}

// ── MJPEG streaming loop ──────────────────────────────────────────────────────

static esp_err_t mjpeg_stream_(httpd_req_t *req) {
  static const char* BOUNDARY = "123456789000000000000987654321";
  static const uint32_t FRAME_MS = 33;   // ~30 fps
  const size_t CHUNK = 8192;
  char hdr[96];

  if (httpd_resp_send_chunk(req, "--", 2) != ESP_OK) return ESP_FAIL;
  if (httpd_resp_send_chunk(req, BOUNDARY, strlen(BOUNDARY)) != ESP_OK) return ESP_FAIL;
  if (httpd_resp_send_chunk(req, "\r\n", 2) != ESP_OK) return ESP_FAIL;

  while (true) {
    const uint32_t t0 = millis();

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }

    uint8_t* jpg_buf; size_t jpg_len; bool must_free;
    if (!get_jpeg_(fb, &jpg_buf, &jpg_len, &must_free)) {
      esp_camera_fb_return(fb); continue;
    }

    int n = snprintf(hdr, sizeof(hdr),
                     "Content-Type: image/jpeg\r\n"
                     "Content-Length: %u\r\n\r\n", (unsigned)jpg_len);
    esp_err_t rc = httpd_resp_send_chunk(req, hdr, n);

    size_t sent = 0;
    while (rc == ESP_OK && sent < jpg_len) {
      size_t to_send = (jpg_len - sent > CHUNK) ? CHUNK : (jpg_len - sent);
      rc = httpd_resp_send_chunk(req, (const char*)jpg_buf + sent, to_send);
      sent += to_send;
    }

    if (must_free) free(jpg_buf);
    esp_camera_fb_return(fb);

    if (rc != ESP_OK) return ESP_FAIL;

    if (httpd_resp_send_chunk(req, "\r\n--", 4) != ESP_OK) return ESP_FAIL;
    if (httpd_resp_send_chunk(req, BOUNDARY, strlen(BOUNDARY)) != ESP_OK) return ESP_FAIL;
    if (httpd_resp_send_chunk(req, "\r\n", 2) != ESP_OK) return ESP_FAIL;

    const uint32_t elapsed = millis() - t0;
    if (elapsed < FRAME_MS) vTaskDelay(pdMS_TO_TICKS(FRAME_MS - elapsed));
  }
}

// ── MJPEG handler ─────────────────────────────────────────────────────────────
// Mutex ensures a rapid reload waits for the previous handler to finish before
// starting a new stream. No camera reinit — stays in YUV422 throughout.

static esp_err_t mjpeg_handler(httpd_req_t *req) {
  if (!s_camMtx || xSemaphoreTake(s_camMtx, pdMS_TO_TICKS(5000)) != pdTRUE) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=123456789000000000000987654321");
  set_cors_no_cache(req);
  httpd_resp_set_hdr(req, "Connection", "keep-alive");

  // Pause AprilTag so the stream has sole access to the camera buffer.
  g_cameraStreaming = true;
  vTaskDelay(pdMS_TO_TICKS(150));

  mjpeg_stream_(req);

  g_cameraStreaming = false;
  xSemaphoreGive(s_camMtx);
  return ESP_OK;
}

// ── Public API ────────────────────────────────────────────────────────────────

bool CameraAPI_begin(uint16_t http_port) {
  if (s_srv) return true;
  if (!s_camMtx) s_camMtx = xSemaphoreCreateMutex();

  httpd_config_t cfg    = HTTPD_DEFAULT_CONFIG();
  cfg.server_port       = http_port;
  cfg.lru_purge_enable  = true;
  cfg.stack_size        = 8192;
  if (httpd_start(&s_srv, &cfg) != ESP_OK) return false;

  httpd_uri_t u_jpg  = { "/camera.jpg", HTTP_GET,     jpg_handler,     nullptr };
  httpd_uri_t u_mjpg = { "/mjpeg",      HTTP_GET,     mjpeg_handler,   nullptr };
  httpd_uri_t u_oj   = { "/camera.jpg", HTTP_OPTIONS, options_handler, nullptr };
  httpd_uri_t u_om   = { "/mjpeg",      HTTP_OPTIONS, options_handler, nullptr };
  httpd_register_uri_handler(s_srv, &u_jpg);
  httpd_register_uri_handler(s_srv, &u_mjpg);
  httpd_register_uri_handler(s_srv, &u_oj);
  httpd_register_uri_handler(s_srv, &u_om);

  Serial.printf("[CameraAPI] MJPEG stream on :%u/mjpeg\n", http_port);
  return true;
}

void CameraAPI_end() {
  if (s_srv) { httpd_stop(s_srv); s_srv = nullptr; }
}
