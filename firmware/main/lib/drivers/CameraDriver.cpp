#include "CameraDriver.h"

static void fill_config_(camera_config_t& c, const CamPins& p,
                         int xclk_hz, bool use_psram) {
  memset(&c, 0, sizeof(c));

  c.pin_pwdn  = p.pin_pwdn;
  c.pin_reset = p.pin_reset;
  c.pin_xclk  = p.pin_xclk;

  // SCCB
  c.pin_sccb_sda = p.pin_siod;
  c.pin_sccb_scl = p.pin_sioc;

  // D0..D7 map to Y2..Y9 (driver expects D0..D7 order)
  c.pin_d0 = p.pin_y2;
  c.pin_d1 = p.pin_y3;
  c.pin_d2 = p.pin_y4;
  c.pin_d3 = p.pin_y5;
  c.pin_d4 = p.pin_y6;
  c.pin_d5 = p.pin_y7;
  c.pin_d6 = p.pin_y8;
  c.pin_d7 = p.pin_y9;

  c.pin_vsync = p.pin_vsync;
  c.pin_href  = p.pin_href;
  c.pin_pclk  = p.pin_pclk;

  c.xclk_freq_hz = xclk_hz;
  c.ledc_timer   = LEDC_TIMER_0;
  c.ledc_channel = LEDC_CHANNEL_0;

  // We stream JPEG; frame size & quality are applied via sensor after init.
  c.pixel_format = PIXFORMAT_JPEG;

  // Buffers
  c.fb_location  = use_psram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  c.grab_mode    = CAMERA_GRAB_LATEST;  // drop old frames if lagging
  c.fb_count     = use_psram ? 2 : 1;   // you can override later if you want
}

bool CameraDriver::begin(const CamPins& pins,
                         int xclk_hz,
                         framesize_t fs,
                         int jpeg_quality,
                         bool use_psram,
                         int fb_count_if_psram) {
  camera_config_t cfg;
  fill_config_(cfg, pins, xclk_hz, use_psram);
  if (use_psram && fb_count_if_psram > 0) cfg.fb_count = fb_count_if_psram;

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    Serial.printf("[Cam] esp_camera_init failed: 0x%x\n", err);
    inited_ = false;
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (!s) {
    Serial.println("[Cam] esp_camera_sensor_get failed");
    esp_camera_deinit();
    inited_ = false;
    return false;
  }

  // Apply initial params
  s->set_framesize(s, fs);
  if (jpeg_quality < 5)  jpeg_quality = 5;
  if (jpeg_quality > 63) jpeg_quality = 63;
  s->set_quality(s, jpeg_quality);
  s->set_hmirror(s, 0);
  s->set_vflip(s, 1);
  inited_ = true;
  return true;
}

bool CameraDriver::setFramesize(framesize_t fs) {
  if (!inited_) return false;
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return false;
  return s->set_framesize(s, fs) == 0;
}

bool CameraDriver::setQuality(int q) {
  if (!inited_) return false;
  if (q < 5)  q = 5;
  if (q > 63) q = 63;
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return false;
  return s->set_quality(s, q) == 0;
}

camera_fb_t* CameraDriver::grab() {
  if (!inited_) return nullptr;
  return esp_camera_fb_get();
}

void CameraDriver::release(camera_fb_t* fb) {
  if (!inited_ || !fb) return;
  esp_camera_fb_return(fb);
}

sensor_t* CameraDriver::sensor() {
  return esp_camera_sensor_get();
}

uint16_t CameraDriver::sensorPID() const {
  sensor_t* s = esp_camera_sensor_get();
  return s ? s->id.PID : 0;
}

void CameraDriver::end() {
  if (!inited_) return;
  esp_camera_deinit();
  inited_ = false;
}
