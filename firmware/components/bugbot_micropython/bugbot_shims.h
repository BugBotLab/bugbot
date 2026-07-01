/*
 * bugbot_shims.h — extern "C" bridge from the pocketpy C extension to the C++ services.
 *
 * Implemented in main/bugbot_shims.cpp (which has access to the C++ service instances).
 * Declared here so bugbot_module.c can call them without knowing about C++.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* C-compatible AprilTag hit — layout matches WebWS::AprilTagHit exactly so
   main.cpp can cast const WebWS::AprilTagHit* -> const BugBotTagHit* safely. */
typedef struct {
    uint8_t id;
    float cx_px, cy_px;   /* tag centre in image pixels (unused by scripts) */
    float az_deg;         /* positive = right of camera centre */
    float el_deg;         /* positive = above camera centre */
    float dist_mm;        /* estimated distance */
    float yaw_deg;        /* tag face rotation (0 = facing camera straight on) */
} BugBotTagHit;

/* C-compatible blob result (from BlobService). */
typedef struct {
    int   cx, cy;         /* centroid in pixel coords */
    int   area;           /* pixel count */
    int   x0, y0, x1, y1; /* bounding box */
    float aspect;         /* (x1-x0)/(y1-y0) */
} BugBotBlob;
#define MAX_BLOB_CACHE 16

/* C-compatible edge/contour result (from ContourService). */
typedef struct {
    int   edge_count;           /* number of edge pixels above threshold */
    float dominant_angle_deg;   /* dominant gradient direction, -90..90 */
} BugBotEdges;

/* C-compatible TinyML result (from TinyMLService). */
#define MAX_TINYML_CLASSES 32
typedef struct {
    float scores[MAX_TINYML_CLASSES]; /* softmax probabilities */
    int   n_classes;
} BugBotMLResult;

/* C-compatible face detection result (from FaceDetectService / HumanFaceDetect). */
#define MAX_FACE_CACHE 8
typedef struct {
    int16_t x1, y1, x2, y2;   /* bounding box in pixel coords */
    float   score;              /* detection confidence 0..1 */
    int16_t kp[10];            /* landmarks: [lx,ly, rx,ry, nx,ny, mlx,mly, mrx,mry] */
} BugBotFace;

/* Forward declarations of C++ service types (used by init functions only). */
struct MotionService;
struct LidarService;
struct ActuatorService;
struct OdomService;
struct PoseBus;
struct AprilTagService;
struct BoardPowerLib;
struct CameraService;
struct BlobService;
struct ContourService;
struct TinyMLService;
struct FaceDetectService;

/* Lifecycle — call once during firmware init to wire up service pointers. */
void bugbot_shims_init(struct MotionService* m, struct ActuatorService* a,
                       struct LidarService* l,  struct PoseBus* p,
                       struct AprilTagService* at, struct BoardPowerLib* bp,
                       struct OdomService* od);

/* Called after bugbot_shims_init() to wire the camera service pointer.
   Separate call so the existing bugbot_module.c binding is unchanged. */
void bugbot_shims_set_camera(struct CameraService* cam);

/* Wire the three CV services and install their result callbacks.
   Call from main.cpp after services are constructed but before scripts run. */
void bugbot_shims_set_cv_services(struct BlobService*    blob,
                                   struct ContourService* contour,
                                   struct TinyMLService*  tinyml);

/* Wire the face detection service and install its result callback.
   Call from main.cpp after bugbot_shims_set_cv_services(). */
void bugbot_shims_set_face_service(struct FaceDetectService* face);

/* CV mode switching — valid modes: "apriltag", "blob", "contour", "tinyml", "face", "none" */
void bugbot_shim_set_cv(const char* mode);

/* Blob results (populated by BlobService callback) */
int  bugbot_shim_blob_count(void);
bool bugbot_shim_blob_get(int idx, BugBotBlob* out);

/* Edge/contour results (populated by ContourService callback) */
bool bugbot_shim_edges_get(BugBotEdges* out);
bool bugbot_shim_edges_valid(void);

/* TinyML classification results (populated by TinyMLService callback) */
bool bugbot_shim_tinyml_get(BugBotMLResult* out);
bool bugbot_shim_tinyml_valid(void);

/* Face detection results (populated by FaceDetectService callback) */
int  bugbot_shim_face_count(void);
bool bugbot_shim_face_get(int idx, BugBotFace* out);
/* Diagnostic: frames where inference ran (0 = detector not loaded or no frames yet) */
int  bugbot_shim_face_frames(void);
/* Diagnostic: 1 if the face detector model is loaded (detector_ non-null), else 0 */
int  bugbot_shim_face_loaded(void);
/* I2C probe of PCA9685 at 0x40: 1=ACK (present), 0=NACK, -1=no motion svc, -2=bus busy */
int  bugbot_shim_motor_ok(void);
/* Direct PCA9685 write bypassing MotionService: spin motor 0 (BR) at 80% for durationMs */
int  bugbot_shim_motor_raw_test(uint32_t durationMs);

/* Called from the AprilTag task callback to cache latest detections for Python. */
void bugbot_shim_on_apriltags(uint8_t count, const BugBotTagHit* hits);

/* Per-script call: ScriptService hands off its stopReq_ flag so wait() can interrupt. */
void bugbot_shim_set_stop_ptr(volatile bool* stop);

/* Motion */
void bugbot_shim_forward(float speed);
void bugbot_shim_backward(float speed);
void bugbot_shim_left(float speed);
void bugbot_shim_right(float speed);
void bugbot_shim_stop(void);
void bugbot_shim_turnl(float speed);
void bugbot_shim_turnr(float speed);

/* Sensors */
float bugbot_shim_distance(void);         /* min of 4-zone LiDAR strip, mm */
bool  bugbot_shim_lidar_grid(uint16_t out[64]); /* full 8x8 grid, row-major */
float bugbot_shim_heading(void);          /* yaw in degrees */
float bugbot_shim_x_mm(void);
float bugbot_shim_y_mm(void);
int   bugbot_shim_battery(void);          /* 0-100 % */
int   bugbot_shim_apriltag_count(void);   /* number of tags in latest scan */
bool  bugbot_shim_apriltag_get(int idx, uint8_t* id,
                                float* cx_px, float* cy_px,
                                float* az, float* el,
                                float* dist, float* yaw);

/* Actuators */
void bugbot_shim_led(uint8_t r, uint8_t g, uint8_t b);
void bugbot_shim_beep(uint32_t freq_hz, uint32_t ms);
void bugbot_shim_servo(uint8_t index, float deg);

/* Heading reset — zeros the IMU yaw offset so forward() moves in the robot's
   current facing direction even when field-centric drive is enabled. */
void bugbot_shim_reset_heading(void);

/* Combined robot-frame drive: fwd/lat/rot each in 0-100 scale, signed. */
void bugbot_shim_drive_body(float fwd, float lat, float rot);

/* Camera power management */
void bugbot_shim_camera_suspend(void); /* deinit + GPIO reset → board cools */
void bugbot_shim_camera_resume(void);  /* re-init + restart detection */

/* Timing / control */
void bugbot_shim_delay_ms(uint32_t ms);
bool bugbot_shim_should_stop(void);

#ifdef __cplusplus
}
#endif
