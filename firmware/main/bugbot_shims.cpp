/*
 * bugbot_shims.cpp — C++ implementation of the extern "C" shims declared in
 * bugbot_shims.h.  Lives in the main component so it has direct access to the
 * service instances without needing extra include paths in bugbot_micropython.
 */

#include "bugbot_shims.h"

#include "lib/services/MotionService.h"
#include "lib/services/LidarService.h"
#include "lib/services/ActuatorService.h"
#include "lib/services/AprilTagService.h"
#include "lib/services/CameraService.h"
#include "lib/services/BlobService.h"
#include "lib/services/ContourService.h"
#include "lib/services/TinyMLService.h"
#include "lib/services/FaceDetectService.h"
#include "lib/services/OdomService.h"
#include "lib/drivers/BoardPowerLib.h"
#include "lib/core/PoseBus.h"
#include "lib/core/DriveDefs.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <string.h>
#include <Wire.h>

static MotionService*   s_motion   = nullptr;
static ActuatorService* s_act      = nullptr;
static LidarService*    s_lidar    = nullptr;
static PoseBus*         s_pose     = nullptr;
static AprilTagService* s_apriltag = nullptr;
static CameraService*   s_camera   = nullptr;
static BlobService*     s_blob     = nullptr;
static ContourService*  s_contour  = nullptr;
static TinyMLService*      s_tinyml   = nullptr;
static FaceDetectService*  s_face     = nullptr;
static BoardPowerLib*   s_power    = nullptr;
static OdomService*     s_odom     = nullptr;
static volatile bool*   s_stop     = nullptr;

/* ── CV result caches (written from camera/inference tasks, read from script task) */

static portMUX_TYPE s_blob_mux   = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_edges_mux  = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_tinyml_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_face_mux   = portMUX_INITIALIZER_UNLOCKED;

static BugBotBlob    s_blob_buf[MAX_BLOB_CACHE];
static uint8_t       s_blob_count = 0;

static BugBotEdges   s_edges      = { 0, 0.0f };
static bool          s_edges_seen = false;

static BugBotMLResult s_tinyml_result  = {};
static bool           s_tinyml_seen    = false;

static BugBotFace s_face_buf[MAX_FACE_CACHE];
static uint8_t    s_face_count = 0;

/* AprilTag cache — written from the AprilTag task, read from ScriptTask. */
#define MAX_TAG_CACHE 8
static portMUX_TYPE s_tag_mux    = portMUX_INITIALIZER_UNLOCKED;
static BugBotTagHit s_tag_buf[MAX_TAG_CACHE];
static uint8_t      s_tag_count  = 0;

extern "C" {

void bugbot_shims_init(MotionService* m, ActuatorService* a,
                       LidarService* l,  PoseBus* p,
                       AprilTagService* at, BoardPowerLib* bp,
                       OdomService* od) {
    s_motion   = m;
    s_act      = a;
    s_lidar    = l;
    s_pose     = p;
    s_apriltag = at;
    s_power    = bp;
    s_odom     = od;
}

void bugbot_shim_on_apriltags(uint8_t count, const BugBotTagHit* hits) {
    if (count > MAX_TAG_CACHE) count = MAX_TAG_CACHE;
    portENTER_CRITICAL(&s_tag_mux);
    s_tag_count = count;
    if (count > 0 && hits) memcpy(s_tag_buf, hits, count * sizeof(BugBotTagHit));
    portEXIT_CRITICAL(&s_tag_mux);
}

void bugbot_shim_set_stop_ptr(volatile bool* stop) {
    s_stop = stop;
    portENTER_CRITICAL(&s_tag_mux);
    s_tag_count = 0;
    portEXIT_CRITICAL(&s_tag_mux);
}

/* ── motion ──────────────────────────────────────────────────────────────── */

void bugbot_shim_forward(float speed) {
    if (s_motion) s_motion->setCommand(DriveDir::Fwd,     speed);
}

/* Returns 1 if PCA9685 at 0x40 ACKs on I2C, 0 if NACK, -1 if no motion svc */
int bugbot_shim_motor_ok(void) {
    if (!s_motion) return -1;
    extern SemaphoreHandle_t g_i2cMutex;
    if (g_i2cMutex && xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(200)) != pdTRUE) return -2;
    Wire.beginTransmission(0x40);
    int rc = Wire.endTransmission();
    if (g_i2cMutex) xSemaphoreGive(g_i2cMutex);
    return (rc == 0) ? 1 : 0;
}

/* Write 4 bytes to PCA9685 LED register block for one channel.
   ON = on-time (0 or 0x1000 for always-on), OFF = off-time (0 or value or 0x1000). */
static void pca9685_set_raw_(uint8_t ch, uint16_t on, uint16_t off) {
    uint8_t reg = 0x06 + 4 * ch;
    Wire.beginTransmission(0x40);
    Wire.write(reg);
    Wire.write((uint8_t)(on & 0xFF));
    Wire.write((uint8_t)(on >> 8));
    Wire.write((uint8_t)(off & 0xFF));
    Wire.write((uint8_t)(off >> 8));
    Wire.endTransmission();
}

/* Bypasses MotionService — direct Wire writes to PCA9685 registers for motor 0 (BR).
   ch5=IN1, ch6=IN2, ch7=PWM. No Adafruit::begin() so Wire timeout is not reset. */
int bugbot_shim_motor_raw_test(uint32_t durationMs) {
    extern SemaphoreHandle_t g_i2cMutex;
    if (g_i2cMutex && xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(500)) != pdTRUE) return -2;
    // Channel 5 (IN1) always-on: ON_H bit4 set = 0x1000, OFF = 0
    pca9685_set_raw_(5, 0x1000, 0);      // IN1 HIGH (always on)
    // Channel 6 (IN2) always-off: ON = 0, OFF_H bit4 set = 0x1000
    pca9685_set_raw_(6, 0, 0x1000);      // IN2 LOW (always off)
    // Channel 7 (PWM) at 80%: OFF = round(0.8 * 4095) = 3276
    pca9685_set_raw_(7, 0, 3276);        // 80% duty
    if (g_i2cMutex) xSemaphoreGive(g_i2cMutex);

    vTaskDelay(pdMS_TO_TICKS(durationMs));

    if (g_i2cMutex && xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(200)) != pdTRUE) return -3;
    pca9685_set_raw_(5, 0, 0x1000);     // IN1 LOW
    pca9685_set_raw_(6, 0, 0x1000);     // IN2 LOW
    pca9685_set_raw_(7, 0, 0x1000);     // PWM off
    if (g_i2cMutex) xSemaphoreGive(g_i2cMutex);
    return 1;
}
void bugbot_shim_backward(float speed) {
    if (s_motion) s_motion->setCommand(DriveDir::Back,    speed);
}
void bugbot_shim_left(float speed) {
    if (s_motion) s_motion->setCommand(DriveDir::StrafeL, speed);
}
void bugbot_shim_right(float speed) {
    if (s_motion) s_motion->setCommand(DriveDir::StrafeR, speed);
}
void bugbot_shim_stop(void) {
    if (s_motion) s_motion->setCommand(DriveDir::Stop,    0.0f);
}
void bugbot_shim_turnl(float speed) {
    if (s_motion) s_motion->setCommand(DriveDir::TurnL,   speed);
}
void bugbot_shim_turnr(float speed) {
    if (s_motion) s_motion->setCommand(DriveDir::TurnR,   speed);
}

/* ── sensors ─────────────────────────────────────────────────────────────── */

float bugbot_shim_distance(void) {
    if (!s_lidar) return 0.0f;
    uint16_t strip[4];
    if (!s_lidar->getStrip(strip)) return 0.0f;
    /* VL53L5CX returns 0 for zones with no valid target — skip zeros. */
    uint16_t best = 0;
    for (int i = 0; i < 4; i++) {
        if (strip[i] && (best == 0 || strip[i] < best)) best = strip[i];
    }
    return (float)best;
}

bool bugbot_shim_lidar_grid(uint16_t out[64]) {
    if (!s_lidar) return false;
    return s_lidar->getGrid(out);
}

float bugbot_shim_heading(void) {
    if (!s_pose) return 0.0f;
    Pose2D p = s_pose->get();
    return p.yaw_rad * (180.0f / (float)M_PI);
}

float bugbot_shim_x_mm(void) { return s_pose ? s_pose->get().x_mm : 0.0f; }
float bugbot_shim_y_mm(void) { return s_pose ? s_pose->get().y_mm : 0.0f; }

int bugbot_shim_battery(void) {
    if (!s_power) return 100;
    /* LiPo: 3.3 V = 0 %, 4.2 V = 100 %, linear. */
    float v   = s_power->readBatteryVolts();
    int   pct = (int)((v - 3.3f) / 0.9f * 100.0f + 0.5f);
    return pct < 0 ? 0 : pct > 100 ? 100 : pct;
}

int bugbot_shim_apriltag_count(void) {
    portENTER_CRITICAL(&s_tag_mux);
    int n = s_tag_count;
    portEXIT_CRITICAL(&s_tag_mux);
    return n;
}

bool bugbot_shim_apriltag_get(int idx, uint8_t* id,
                               float* cx_px, float* cy_px,
                               float* az, float* el,
                               float* dist, float* yaw) {
    portENTER_CRITICAL(&s_tag_mux);
    bool ok = (idx >= 0 && idx < (int)s_tag_count);
    if (ok) {
        *id    = s_tag_buf[idx].id;
        *cx_px = s_tag_buf[idx].cx_px;
        *cy_px = s_tag_buf[idx].cy_px;
        *az    = s_tag_buf[idx].az_deg;
        *el    = s_tag_buf[idx].el_deg;
        *dist  = s_tag_buf[idx].dist_mm;
        *yaw   = s_tag_buf[idx].yaw_deg;
    }
    portEXIT_CRITICAL(&s_tag_mux);
    return ok;
}

/* ── actuators ───────────────────────────────────────────────────────────── */

void bugbot_shim_led(uint8_t r, uint8_t g, uint8_t b) {
    if (s_act) s_act->setLed(r, g, b);
}

void bugbot_shim_beep(uint32_t freq_hz, uint32_t ms) {
    if (!s_act) return;
    if (freq_hz > 0) s_act->setBuzzerTone(freq_hz);
    else             s_act->buzzerOff();
    if (ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(ms));
        s_act->buzzerOff();
    }
}

void bugbot_shim_servo(uint8_t index, float deg) {
    if (!s_act) return;
    if (index == 1) s_act->setServo1Deg(deg);
    else            s_act->setServo2Deg(deg);
}

/* ── heading reset ───────────────────────────────────────────────────────── */

void bugbot_shim_reset_heading(void) {
    if (s_odom) s_odom->correctYawRad(0.0f);
}

void bugbot_shim_drive_body(float fwd, float lat, float rot) {
    if (s_motion) s_motion->setCommandVec(fwd * 0.01f, lat * 0.01f, rot * 0.01f);
}

/* ── camera power ────────────────────────────────────────────────────────── */

void bugbot_shim_camera_suspend(void) {
    if (s_camera) s_camera->suspend();
}

void bugbot_shim_camera_resume(void) {
    if (s_camera) s_camera->resume();
}

void bugbot_shims_set_camera(CameraService* cam) {
    s_camera = cam;
}

void bugbot_shims_set_cv_services(BlobService*    blob,
                                   ContourService* contour,
                                   TinyMLService*  tinyml) {
    s_blob    = blob;
    s_contour = contour;
    s_tinyml  = tinyml;

    if (blob) {
        blob->setCallback([](const BlobService::Blob* blobs, int count) {
            portENTER_CRITICAL(&s_blob_mux);
            s_blob_count = (uint8_t)((count > MAX_BLOB_CACHE) ? MAX_BLOB_CACHE : count);
            for (int i = 0; i < s_blob_count; i++) {
                s_blob_buf[i].cx     = blobs[i].cx;
                s_blob_buf[i].cy     = blobs[i].cy;
                s_blob_buf[i].area   = blobs[i].area;
                s_blob_buf[i].x0     = blobs[i].x0;
                s_blob_buf[i].y0     = blobs[i].y0;
                s_blob_buf[i].x1     = blobs[i].x1;
                s_blob_buf[i].y1     = blobs[i].y1;
                s_blob_buf[i].aspect = blobs[i].aspect;
            }
            portEXIT_CRITICAL(&s_blob_mux);
        });
    }

    if (contour) {
        contour->setCallback([](const ContourService::EdgeResult& r) {
            portENTER_CRITICAL(&s_edges_mux);
            s_edges.edge_count          = r.edgeCount;
            s_edges.dominant_angle_deg  = r.dominantAngleDeg;
            s_edges_seen = true;
            portEXIT_CRITICAL(&s_edges_mux);
        });
    }

    if (tinyml) {
        tinyml->setCallback([](const float* scores, int nClasses) {
            portENTER_CRITICAL(&s_tinyml_mux);
            const int n = (nClasses > MAX_TINYML_CLASSES) ? MAX_TINYML_CLASSES : nClasses;
            for (int i = 0; i < n; i++) s_tinyml_result.scores[i] = scores[i];
            s_tinyml_result.n_classes = n;
            s_tinyml_seen = true;
            portEXIT_CRITICAL(&s_tinyml_mux);
        });
    }
}

void bugbot_shims_set_face_service(FaceDetectService* face) {
    s_face = face;
    if (face) {
        face->setCallback([](const FaceDetectService::Face* faces, int count) {
            portENTER_CRITICAL(&s_face_mux);
            s_face_count = (uint8_t)((count > MAX_FACE_CACHE) ? MAX_FACE_CACHE : count);
            for (int i = 0; i < s_face_count; i++) {
                s_face_buf[i].x1    = faces[i].x1;
                s_face_buf[i].y1    = faces[i].y1;
                s_face_buf[i].x2    = faces[i].x2;
                s_face_buf[i].y2    = faces[i].y2;
                s_face_buf[i].score = faces[i].score;
                for (int k = 0; k < 10; k++) s_face_buf[i].kp[k] = faces[i].kp[k];
            }
            portEXIT_CRITICAL(&s_face_mux);
        });
    }
}

/* ── CV mode switching ───────────────────────────────────────────────────────── */

void bugbot_shim_set_cv(const char* mode) {
    if (!s_camera || !mode) return;
    if (strcmp(mode, "apriltag") == 0)      s_camera->setConsumer(s_apriltag);
    else if (strcmp(mode, "blob")    == 0)  s_camera->setConsumer(s_blob);
    else if (strcmp(mode, "contour") == 0)  s_camera->setConsumer(s_contour);
    else if (strcmp(mode, "tinyml")  == 0)  s_camera->setConsumer(s_tinyml);
    else if (strcmp(mode, "face")    == 0)  s_camera->setConsumer(s_face);
    else if (strcmp(mode, "none")    == 0)  s_camera->setConsumer(nullptr);
}

/* ── blob results ────────────────────────────────────────────────────────────── */

int bugbot_shim_blob_count(void) {
    portENTER_CRITICAL(&s_blob_mux);
    int n = s_blob_count;
    portEXIT_CRITICAL(&s_blob_mux);
    return n;
}

bool bugbot_shim_blob_get(int idx, BugBotBlob* out) {
    portENTER_CRITICAL(&s_blob_mux);
    bool ok = (idx >= 0 && idx < (int)s_blob_count);
    if (ok && out) *out = s_blob_buf[idx];
    portEXIT_CRITICAL(&s_blob_mux);
    return ok;
}

/* ── edge/contour results ────────────────────────────────────────────────────── */

bool bugbot_shim_edges_get(BugBotEdges* out) {
    portENTER_CRITICAL(&s_edges_mux);
    bool ok = s_edges_seen;
    if (ok && out) *out = s_edges;
    portEXIT_CRITICAL(&s_edges_mux);
    return ok;
}

bool bugbot_shim_edges_valid(void) {
    portENTER_CRITICAL(&s_edges_mux);
    bool ok = s_edges_seen;
    portEXIT_CRITICAL(&s_edges_mux);
    return ok;
}

/* ── TinyML results ──────────────────────────────────────────────────────────── */

bool bugbot_shim_tinyml_get(BugBotMLResult* out) {
    portENTER_CRITICAL(&s_tinyml_mux);
    bool ok = s_tinyml_seen;
    if (ok && out) *out = s_tinyml_result;
    portEXIT_CRITICAL(&s_tinyml_mux);
    return ok;
}

bool bugbot_shim_tinyml_valid(void) {
    portENTER_CRITICAL(&s_tinyml_mux);
    bool ok = s_tinyml_seen;
    portEXIT_CRITICAL(&s_tinyml_mux);
    return ok;
}

/* ── face detection results ──────────────────────────────────────────────────── */

int bugbot_shim_face_count(void) {
    portENTER_CRITICAL(&s_face_mux);
    int n = s_face_count;
    portEXIT_CRITICAL(&s_face_mux);
    return n;
}

bool bugbot_shim_face_get(int idx, BugBotFace* out) {
    portENTER_CRITICAL(&s_face_mux);
    bool ok = (idx >= 0 && idx < (int)s_face_count);
    if (ok && out) *out = s_face_buf[idx];
    portEXIT_CRITICAL(&s_face_mux);
    return ok;
}

int bugbot_shim_face_frames(void) {
    if (s_camera && !s_camera->isInited()) return -1; // camera reinit failed
    return s_face ? s_face->frameCount() : 0;
}

int bugbot_shim_face_loaded(void) {
    return (s_face && s_face->detectorReady()) ? 1 : 0;
}

/* ── timing / control ────────────────────────────────────────────────────── */

void bugbot_shim_delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms ? ms : 1));
}

bool bugbot_shim_should_stop(void) {
    return s_stop && *s_stop;
}

} /* extern "C" */
