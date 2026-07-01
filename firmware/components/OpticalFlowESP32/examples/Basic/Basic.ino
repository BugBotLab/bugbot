#include <Arduino.h>
#include <SPI.h>
#include <OpticalFlowESP32.h>

// ===== Wiring (XIAO ESP32-S3 example) =====
constexpr int PIN_SCK  = 7;   // D8
constexpr int PIN_MISO = 8;   // D9
constexpr int PIN_MOSI = 9;   // D10
constexpr int PIN_CS_F = 4;   // Front sensor CS (example)
constexpr int PIN_CS_B = 2;   // Back  sensor CS (example)

// ===== Model / Orientation =====
constexpr auto MODEL   = OpticalFlowESP32::PMW3901; // or PAA5100
constexpr char ORIENTF = 'N';   // FRONT  sensor faces forward
constexpr char ORIENTB = 'N';   // BACK   sensor also faces forward

// ===== Geometry & scaling =====
constexpr float BASELINE_MM       = 34.0f;  // front <-> back spacing (mm)
constexpr float MM_PER_COUNT      = 0.03;  // tune for 20 mm height & surface

// ===== Pose data =====
struct Pose2D { float x_mm = 0, y_mm = 0, yaw_rad = 0; };
Pose2D pose;

// ===== SPI & sensors (single shared bus) =====
SPIClass hspi(1);
OpticalFlowESP32 flowFront(hspi);
OpticalFlowESP32 flowBack (hspi);

// ===== Odometry for FRONT/BACK sensors (both 'N') =====
// +x forward, +y left; yaw CCW+ (radians). Small-angle update.
inline void updatePoseFrontBack(
  int16_t dxF, int16_t dyF,       // front sensor counts
  int16_t dxB, int16_t dyB,       // back  sensor counts
  Pose2D& pose,                   // in/out: integrates here
  float mm_per_count = 0.03f,     // tune me
  float baseline_mm   = 34.0f     // front↔back spacing
) {
  // 1) Counts -> body-frame millimetres
  const float fF = mm_per_count * dyF;   // forward from dy
  const float lF = mm_per_count * dxF;  // left from -dx
  const float fB = mm_per_count * dyB;
  const float lB = mm_per_count * dxB;

  // 2) Body-frame translation + yaw
  const float dxb  = 0.5f * (fF + fB);           // forward
  const float dyb  = 0.5f * (lF + lB);           // left
  const float dyaw = (lF - lB) / baseline_mm;    // radians

  // 3) Rotate into world and integrate
  const float c = cosf(pose.yaw_rad), s = sinf(pose.yaw_rad);
  pose.x_mm  +=  c * dxb - s * dyb;
  pose.y_mm  +=  s * dxb + c * dyb;
  pose.yaw_rad += dyaw;

  // (optional) wrap yaw
  if (pose.yaw_rad >  M_PI) pose.yaw_rad -= 2.0f * M_PI;
  if (pose.yaw_rad < -M_PI) pose.yaw_rad += 2.0f * M_PI;
}

void setup() {
  Serial.begin(115200);
  delay(50);

  // Init SPI once (no CS)
  hspi.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
  hspi.setHwCs(false);

  pinMode(PIN_CS_F, OUTPUT); digitalWrite(PIN_CS_F, HIGH);
  pinMode(PIN_CS_B, OUTPUT); digitalWrite(PIN_CS_B, HIGH);
  delay(2);

  bool okF = flowFront.begin(hspi, PIN_CS_F, MODEL, ORIENTF);
  bool okB = flowBack .begin(hspi, PIN_CS_B, MODEL, ORIENTB);
  Serial.printf("begin: front=%d back=%d\n", okF, okB);

  // Optional: turn on LEDs if supported
  flowFront.setLed(true);
  flowBack .setLed(true);
}

float front_x = 0, front_y = 0;
float back_x  = 0, back_y  = 0;

void loop() {
  // 1) Read incremental motion (counts) from each sensor
  int16_t dxF=0, dyF=0, dxB=0, dyB=0;
  flowFront.readMotionSimple(dxF, dyF);
  flowBack .readMotionSimple(dxB, dyB);

  // 2) Update absolute pose
  updatePoseFrontBack(-dxF, -dyF, dxB, dyB, pose);

  front_x += -dxF * MM_PER_COUNT;
  front_y += -dyF * MM_PER_COUNT;
  back_x  += dxB * MM_PER_COUNT;
  back_y  += dyB * MM_PER_COUNT;

  // Print absolute totals for each sensor
  //Serial.printf("Front: X=%.1f mm  Y=%.1f mm | Back: X=%.1f mm  Y=%.1f mm\n",
  //              front_x, front_y, back_x, back_y);


  // 3) Print absolute pose
  Serial.printf("ABS POS: X=%.1f mm  Y=%.1f mm  Yaw=%.2f deg\n",
                pose.x_mm, pose.y_mm, pose.yaw_rad * 180.0f / M_PI);

  // 4) Serial command: 'r' to reset pose
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      pose = Pose2D();  // reset to (0,0,0)
        front_x = 0;
        front_y = 0;
        back_x  = 0;
        back_y  = 0;


      Serial.println(F("Pose reset to zero."));
    }
  }

  delay(10); // ~100 Hz
}
