// CameraDriver.h
// OV5640 camera driver wrapping esp_camera. Manages init, frame grab/release,
// and runtime resolution/quality changes. Default pins match XIAO ESP32-S3 Sense.
#pragma once
#include <Arduino.h>
#include <esp_camera.h>

// Xiao ESP32S3 Sense (OV5640) default plug pins
struct CamPins {
  int pin_pwdn   = -1;
  int pin_reset  = -1;
  int pin_xclk   = 10;
  int pin_siod   = 40; // SCCB SDA
  int pin_sioc   = 39; // SCCB SCL
  int pin_y2     = 15; // D0
  int pin_y3     = 17; // D1
  int pin_y4     = 18; // D2
  int pin_y5     = 16; // D3
  int pin_y6     = 14; // D4
  int pin_y7     = 12; // D5
  int pin_y8     = 11; // D6
  int pin_y9     = 48; // D7
  int pin_vsync  = 38;
  int pin_href   = 47;
  int pin_pclk   = 13;
};

// XIAO ESP32S3 Sense default plug (OV5640)
inline CamPins DefaultXiaoSensePins() { return CamPins{}; }

class CameraDriver {
public:
  // Bring-up (PIXFORMAT_JPEG internally). Tweak fs/quality later with setters if needed.
  bool begin(const CamPins& pins = DefaultXiaoSensePins(),
             int xclk_hz = 20'000'000,
             framesize_t fs = FRAMESIZE_QQVGA,
             int jpeg_quality = 22,
             bool use_psram = true,
             int fb_count_if_psram = 2);

  // Change parameters at runtime
  bool setFramesize(framesize_t fs);
  bool setQuality(int q);

  // Frame I/O
  camera_fb_t* grab();                  // esp_camera_fb_get()
  inline camera_fb_t* capture() { return grab(); } // alias expected by CameraService
  void         release(camera_fb_t* fb);            // esp_camera_fb_return(fb)

  // Compatibility aliases expected by CameraService
  inline bool setResolution(framesize_t fs) { return setFramesize(fs); }
  inline bool setJpegQuality(int q)         { return setQuality(q); }

  // Sensor helpers
  sensor_t*  sensor();
  uint16_t   sensorPID() const;

  // Safe shutdown (optional)
  void end();

private:
  bool inited_ = false;
};
