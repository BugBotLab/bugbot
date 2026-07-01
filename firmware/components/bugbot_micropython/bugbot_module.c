/*
 * bugbot_module.c — pocketpy C extension that exposes robot control functions.
 *
 * Registers a Python module named "bugbot" with:
 *   forward([speed=50])   backward([speed=50])  left([speed=50])  right([speed=50])
 *   stop()                turn(degrees[, speed=30])  wait(seconds)
 *   distance()            lidar_grid()          heading()         battery()
 *   position()            apriltags()
 *   led(r, g, b)          beep(freq[, ms=200])  servo(index, deg)
 *   set_cv(mode)          camera_suspend()  camera_resume()
 *   blobs()               edges()           tinyml_result()   faces()
 *   go()                  <- injects all the above into __main__ globals
 *
 * set_cv() modes: "apriltag" | "blob" | "contour" | "tinyml" | "face" | "none"
 * blobs()  -> [[cx, cy, area, x0, y0, x1, y1, aspect], ...]  (BlobService)
 * edges()  -> [edge_count, dominant_angle_deg]               (ContourService)
 * tinyml_result() -> [score0, score1, ...]                   (TinyMLService)
 * faces()  -> [[x1,y1,x2,y2,score,[kp0..kp9]], ...]         (FaceDetectService)
 *
 * turn() sign convention: positive degrees = right (clockwise).
 *
 * Student script pattern (same file works for proxy AND upload mode):
 *   import bugbot; bugbot.go()
 *   forward(50); wait(1); stop()
 *   print("dist:", distance())
 *   grid = lidar_grid()          # list of 64 ints (mm), row-major 8x8
 *   for tag in apriltags():      # list of [id, az_deg, el_deg, dist_mm, yaw_deg]
 *       print(tag[0], tag[3])    # id, distance
 *   x, y = position()
 */

#include "pocketpy.h"
#include "bugbot_shims.h"
#include <math.h>

#define TURN_SPEED      30.0f
#define DEFAULT_SPEED   50.0f

/* ── helpers ──────────────────────────────────────────────────────────────── */

/* Raise RuntimeError("BugBot: script stopped") and return false. */
static bool raise_stopped(void) {
    return RuntimeError("BugBot: script stopped");
}

/* Parse a float from arg[i], accepting both int and float Python types. */
static bool get_float(py_StackRef argv, int i, float* out) {
    py_Ref a = py_arg(i);
    if (py_isint(a))   { *out = (float)py_toint(a);   return true; }
    if (py_isfloat(a)) { *out = (float)py_tofloat(a); return true; }
    return TypeError("expected a number, got %t", a->type);
}

/* ── motion ───────────────────────────────────────────────────────────────── */

static bool bb_forward(int argc, py_StackRef argv) {
    if (argc > 1) return TypeError("forward([speed]) takes 0 or 1 arguments");
    float sp = DEFAULT_SPEED;
    if (argc == 1 && !get_float(argv, 0, &sp)) return false;
    if (bugbot_shim_should_stop()) return raise_stopped();
    bugbot_shim_forward(sp * 0.01f);
    py_newnone(py_retval());
    return true;
}

static bool bb_backward(int argc, py_StackRef argv) {
    if (argc > 1) return TypeError("backward([speed]) takes 0 or 1 arguments");
    float sp = DEFAULT_SPEED;
    if (argc == 1 && !get_float(argv, 0, &sp)) return false;
    if (bugbot_shim_should_stop()) return raise_stopped();
    bugbot_shim_backward(sp * 0.01f);
    py_newnone(py_retval());
    return true;
}

static bool bb_left(int argc, py_StackRef argv) {
    if (argc > 1) return TypeError("left([speed]) takes 0 or 1 arguments");
    float sp = DEFAULT_SPEED;
    if (argc == 1 && !get_float(argv, 0, &sp)) return false;
    if (bugbot_shim_should_stop()) return raise_stopped();
    bugbot_shim_left(sp * 0.01f);
    py_newnone(py_retval());
    return true;
}

static bool bb_right(int argc, py_StackRef argv) {
    if (argc > 1) return TypeError("right([speed]) takes 0 or 1 arguments");
    float sp = DEFAULT_SPEED;
    if (argc == 1 && !get_float(argv, 0, &sp)) return false;
    if (bugbot_shim_should_stop()) return raise_stopped();
    bugbot_shim_right(sp * 0.01f);
    py_newnone(py_retval());
    return true;
}

static bool bb_stop(int argc, py_StackRef argv) {
    (void)argc; (void)argv;
    bugbot_shim_stop();
    py_newnone(py_retval());
    return true;
}

/* spin_right(speed) / spin_left(speed): non-blocking continuous turn.
   Unlike turn(), these do NOT use the IMU — call stop() or the next
   motion command to end the spin. Useful when the IMU is unavailable. */
static bool bb_spin_right(int argc, py_StackRef argv) {
    if (argc > 1) return TypeError("spin_right([speed]) takes 0 or 1 arguments");
    float sp = DEFAULT_SPEED;
    if (argc == 1 && !get_float(argv, 0, &sp)) return false;
    if (bugbot_shim_should_stop()) return raise_stopped();
    bugbot_shim_turnr(sp * 0.01f);
    py_newnone(py_retval());
    return true;
}

static bool bb_spin_left(int argc, py_StackRef argv) {
    if (argc > 1) return TypeError("spin_left([speed]) takes 0 or 1 arguments");
    float sp = DEFAULT_SPEED;
    if (argc == 1 && !get_float(argv, 0, &sp)) return false;
    if (bugbot_shim_should_stop()) return raise_stopped();
    bugbot_shim_turnl(sp * 0.01f);
    py_newnone(py_retval());
    return true;
}

/* turn(degrees, speed=30): positive = right (clockwise), negative = left.
 * Uses IMU heading for closed-loop control; safety timeout fires if the IMU
 * is not yet updating (e.g. first second after boot). */
static bool bb_turn(int argc, py_StackRef argv) {
    if (argc < 1 || argc > 2) return TypeError("turn(degrees[, speed]) takes 1 or 2 arguments");
    float deg; if (!get_float(argv, 0, &deg)) return false;
    if (bugbot_shim_should_stop()) return raise_stopped();

    float spd = TURN_SPEED;
    if (argc == 2 && !get_float(argv, 1, &spd)) return false;

    float target  = deg < 0.0f ? -deg : deg;
    /* Safety timeout: 4× the time it would take at 15 deg/s (very conservative). */
    uint32_t maxMs = (uint32_t)(target / 15.0f * 1000.0f) * 4;
    if (maxMs < 500) maxMs = 500;

    if (deg > 0.0f) bugbot_shim_turnr(spd * 0.01f);
    else            bugbot_shim_turnl(spd * 0.01f);

    float    prev    = bugbot_shim_heading();
    float    total   = 0.0f;
    uint32_t elapsed = 0;

    while (total < target - 2.0f && elapsed < maxMs) {
        bugbot_shim_delay_ms(10);
        elapsed += 10;
        if (bugbot_shim_should_stop()) {
            bugbot_shim_stop();
            return raise_stopped();
        }
        float curr  = bugbot_shim_heading();
        float delta = curr - prev;
        if      (delta >  180.0f) delta -= 360.0f;
        else if (delta < -180.0f) delta += 360.0f;
        total += delta < 0.0f ? -delta : delta;
        prev = curr;
    }

    bugbot_shim_stop();
    py_newnone(py_retval());
    return true;
}

/* wait(seconds): yields the FreeRTOS task; checks stop every 10 ms. */
static bool bb_wait(int argc, py_StackRef argv) {
    if (argc != 1) return TypeError("wait() takes 1 argument");
    float secs; if (!get_float(argv, 0, &secs)) return false;
    if (bugbot_shim_should_stop()) return raise_stopped();

    uint32_t ms = (uint32_t)(secs * 1000.0f);
    uint32_t elapsed = 0;
    while (elapsed < ms) {
        uint32_t chunk = (ms - elapsed > 10) ? 10 : (ms - elapsed);
        bugbot_shim_delay_ms(chunk);
        elapsed += chunk;
        if (bugbot_shim_should_stop()) return raise_stopped();
    }
    py_newnone(py_retval());
    return true;
}

/* ── sensors ──────────────────────────────────────────────────────────────── */

static bool bb_distance(int argc, py_StackRef argv) {
    (void)argc; (void)argv;
    py_newfloat(py_retval(), (py_f64)bugbot_shim_distance());
    return true;
}

/* lidar_grid() -> list of 64 ints (mm), row-major 8×8.
   Row 0 = far side of robot, Row 7 = near side (sensor faces forward). */
static bool bb_lidar_grid(int argc, py_StackRef argv) {
    (void)argc; (void)argv;
    uint16_t grid[64];
    bool ok = bugbot_shim_lidar_grid(grid);
    py_newlistn(py_retval(), 64);
    py_Ref lst = py_retval();
    for (int i = 0; i < 64; i++) {
        py_newint(py_list_getitem(lst, i), ok ? (py_i64)grid[i] : (py_i64)0);
    }
    return true;
}

static bool bb_heading(int argc, py_StackRef argv) {
    (void)argc; (void)argv;
    py_newfloat(py_retval(), (py_f64)bugbot_shim_heading());
    return true;
}

/* position() -> [x_mm, y_mm] from the fused pose estimate. */
static bool bb_position(int argc, py_StackRef argv) {
    (void)argc; (void)argv;
    py_newlistn(py_retval(), 2);
    py_Ref lst = py_retval();
    py_newfloat(py_list_getitem(lst, 0), (py_f64)bugbot_shim_x_mm());
    py_newfloat(py_list_getitem(lst, 1), (py_f64)bugbot_shim_y_mm());
    return true;
}

static bool bb_battery(int argc, py_StackRef argv) {
    (void)argc; (void)argv;
    py_newint(py_retval(), (py_i64)bugbot_shim_battery());
    return true;
}

/* apriltags() -> list of [id, cx_px, cy_px, dist_mm] per detected tag.
   id:      tag ID integer.
   cx_px:   tag centre X in image pixels (0-159 for QQVGA, centre = 80).
   cy_px:   tag centre Y in image pixels (0-119 for QQVGA, centre = 60).
   dist_mm: distance estimate from tag apparent pixel size (EMA-smoothed). */
static bool bb_apriltags(int argc, py_StackRef argv) {
    (void)argc; (void)argv;
    int count = bugbot_shim_apriltag_count();
    py_newlistn(py_retval(), count);
    py_Ref outer = py_retval();
    for (int i = 0; i < count; i++) {
        uint8_t id; float cx, cy, az, el, dist, yaw;
        if (!bugbot_shim_apriltag_get(i, &id, &cx, &cy, &az, &el, &dist, &yaw)) continue;
        /* Build inner list [id, cx_px, cy_px, dist_mm] at outer[i].
           dist_mm is estimated from tag apparent pixel size — use for approach
           stop condition (LiDAR strip reads floor, not the tag). */
        py_newlistn(py_list_getitem(outer, i), 4);
        py_Ref inner = py_list_getitem(outer, i);
        py_newint  (py_list_getitem(inner, 0), (py_i64)id);
        py_newfloat(py_list_getitem(inner, 1), (py_f64)cx);
        py_newfloat(py_list_getitem(inner, 2), (py_f64)cy);
        py_newfloat(py_list_getitem(inner, 3), (py_f64)dist);
    }
    return true;
}

/* reset_heading(): zero the IMU yaw offset so forward() moves in the robot's
   current facing direction regardless of accumulated field-centric heading. */
static bool bb_reset_heading(int argc, py_StackRef argv) {
    (void)argc; (void)argv;
    bugbot_shim_reset_heading();
    py_newnone(py_retval());
    return true;
}

/* drive(fwd, lat, rot=0): combined robot-frame motion.
   fwd/lat/rot each -100 to +100.  Positive lat = strafe right. */
static bool bb_drive(int argc, py_StackRef argv) {
    if (argc < 2 || argc > 3) return TypeError("drive(fwd, lat[, rot]) takes 2 or 3 arguments");
    float fwd, lat, rot = 0.0f;
    if (!get_float(argv, 0, &fwd)) return false;
    if (!get_float(argv, 1, &lat)) return false;
    if (argc == 3 && !get_float(argv, 2, &rot)) return false;
    if (bugbot_shim_should_stop()) return raise_stopped();
    bugbot_shim_drive_body(fwd, lat, rot);
    py_newnone(py_retval());
    return true;
}

/* ── actuators ────────────────────────────────────────────────────────────── */

/* led(r, g, b)  OR  led("red") etc. — colour strings not supported here
   (proxy-mode _proxy.py handles them on the PC side; on robot side pass numbers). */
static bool bb_led(int argc, py_StackRef argv) {
    if (argc != 3) return TypeError("led(r, g, b) takes 3 arguments");
    float r, g, b;
    if (!get_float(argv, 0, &r)) return false;
    if (!get_float(argv, 1, &g)) return false;
    if (!get_float(argv, 2, &b)) return false;
    bugbot_shim_led((uint8_t)r, (uint8_t)g, (uint8_t)b);
    py_newnone(py_retval());
    return true;
}

/* beep(freq, ms=200) */
static bool bb_beep(int argc, py_StackRef argv) {
    if (argc < 1 || argc > 2) return TypeError("beep(freq[, ms]) takes 1 or 2 arguments");
    float freq; if (!get_float(argv, 0, &freq)) return false;
    float ms_f = 200.0f;
    if (argc == 2 && !get_float(argv, 1, &ms_f)) return false;
    bugbot_shim_beep((uint32_t)freq, (uint32_t)ms_f);
    py_newnone(py_retval());
    return true;
}

/* servo(index, angle_deg) */
static bool bb_servo(int argc, py_StackRef argv) {
    if (argc != 2) return TypeError("servo(index, deg) takes 2 arguments");
    float idx, deg;
    if (!get_float(argv, 0, &idx)) return false;
    if (!get_float(argv, 1, &deg)) return false;
    bugbot_shim_servo((uint8_t)idx, deg);
    py_newnone(py_retval());
    return true;
}

/* ── camera power ─────────────────────────────────────────────────────────── */

static bool bb_camera_suspend(int argc, py_StackRef argv) {
    (void)argc; (void)argv;
    bugbot_shim_camera_suspend();
    py_newnone(py_retval());
    return true;
}

static bool bb_camera_resume(int argc, py_StackRef argv) {
    (void)argc; (void)argv;
    bugbot_shim_camera_resume();
    py_newnone(py_retval());
    return true;
}

/* ── CV service switching and results ────────────────────────────────────────

   set_cv(mode)  — switch the active computer-vision consumer.
                   mode: "apriltag" | "blob" | "contour" | "tinyml" | "none"

   blobs()       — list of blob dicts from BlobService.
                   Each blob: [cx, cy, area, x0, y0, x1, y1, aspect]
                   Returns [] if BlobService is not the active consumer or no
                   blobs have been detected yet.

   edges()       — [edge_count, dominant_angle_deg] from ContourService.
                   Returns [0, 0.0] if ContourService has not produced a result.

   tinyml_result() — list of float class scores from TinyMLService.
                   Returns [] if no inference has completed yet.
*/

static bool bb_set_cv(int argc, py_StackRef argv) {
    if (argc != 1) return TypeError("set_cv(mode) takes 1 argument");
    py_Ref a = py_arg(0);
    if (!py_isstr(a)) return TypeError("set_cv(mode): mode must be a string");
    bugbot_shim_set_cv(py_tostr(a));
    py_newnone(py_retval());
    return true;
}

/* blobs() -> [[cx, cy, area, x0, y0, x1, y1, aspect], ...] */
static bool bb_blobs(int argc, py_StackRef argv) {
    (void)argc; (void)argv;
    int count = bugbot_shim_blob_count();
    py_newlistn(py_retval(), count);
    py_Ref outer = py_retval();
    for (int i = 0; i < count; i++) {
        BugBotBlob b;
        if (!bugbot_shim_blob_get(i, &b)) continue;
        py_newlistn(py_list_getitem(outer, i), 8);
        py_Ref inner = py_list_getitem(outer, i);
        py_newint  (py_list_getitem(inner, 0), (py_i64)b.cx);
        py_newint  (py_list_getitem(inner, 1), (py_i64)b.cy);
        py_newint  (py_list_getitem(inner, 2), (py_i64)b.area);
        py_newint  (py_list_getitem(inner, 3), (py_i64)b.x0);
        py_newint  (py_list_getitem(inner, 4), (py_i64)b.y0);
        py_newint  (py_list_getitem(inner, 5), (py_i64)b.x1);
        py_newint  (py_list_getitem(inner, 6), (py_i64)b.y1);
        py_newfloat(py_list_getitem(inner, 7), (py_f64)b.aspect);
    }
    return true;
}

/* edges() -> [edge_count, dominant_angle_deg] */
static bool bb_edges(int argc, py_StackRef argv) {
    (void)argc; (void)argv;
    BugBotEdges e;
    if (!bugbot_shim_edges_get(&e)) {
        e.edge_count = 0;
        e.dominant_angle_deg = 0.0f;
    }
    py_newlistn(py_retval(), 2);
    py_Ref lst = py_retval();
    py_newint  (py_list_getitem(lst, 0), (py_i64)e.edge_count);
    py_newfloat(py_list_getitem(lst, 1), (py_f64)e.dominant_angle_deg);
    return true;
}

/* tinyml_result() -> [score0, score1, ...] or [] if no inference yet */
static bool bb_tinyml_result(int argc, py_StackRef argv) {
    (void)argc; (void)argv;
    BugBotMLResult r;
    if (!bugbot_shim_tinyml_get(&r)) {
        py_newlistn(py_retval(), 0);
        return true;
    }
    py_newlistn(py_retval(), r.n_classes);
    py_Ref lst = py_retval();
    for (int i = 0; i < r.n_classes; i++) {
        py_newfloat(py_list_getitem(lst, i), (py_f64)r.scores[i]);
    }
    return true;
}

/* face_frames() -> int: number of inference frames since set_cv("face").
   0 = model not loaded or no frames received yet. */
static bool bb_face_frames(int argc, py_StackRef argv) {
    (void)argc; (void)argv;
    py_newint(py_retval(), (py_i64)bugbot_shim_face_frames());
    return true;
}

/* face_loaded() -> int: 1 if MSRMNP model was loaded from LittleFS, else 0.
   If 0 after set_cv("face"), the model files are missing (LittleFS problem).
   If 1 but face_frames()==0, model loaded but camera not delivering frames. */
static bool bb_face_loaded(int argc, py_StackRef argv) {
    (void)argc; (void)argv;
    py_newint(py_retval(), (py_i64)bugbot_shim_face_loaded());
    return true;
}

/* motor_ok() -> int: 1=PCA9685 ACK on I2C, 0=NACK, -1=no motion svc, -2=bus busy */
static bool bb_motor_ok(int argc, py_StackRef argv) {
    (void)argc; (void)argv;
    py_newint(py_retval(), (py_i64)bugbot_shim_motor_ok());
    return true;
}

/* motor_raw_test([ms=2000]) -> 1 on success. Bypasses MotionService entirely.
   If this moves the motor but forward() doesn't, the issue is MotionService config.
   If this also fails, the issue is hardware (wiring/driver). */
static bool bb_motor_raw_test(int argc, py_StackRef argv) {
    float ms = 2000.0f;
    if (argc >= 1 && !get_float(argv, 0, &ms)) return false;
    py_newint(py_retval(), (py_i64)bugbot_shim_motor_raw_test((uint32_t)ms));
    return true;
}

/* faces() -> [[x1, y1, x2, y2, score, [kp0..kp9]], ...] or [] if no faces.
   kp[0..9] = [lx,ly, rx,ry, nx,ny, mlx,mly, mrx,mry] (5 face landmarks). */
static bool bb_faces(int argc, py_StackRef argv) {
    (void)argc; (void)argv;
    int count = bugbot_shim_face_count();
    py_newlistn(py_retval(), count);
    py_Ref outer = py_retval();
    for (int i = 0; i < count; i++) {
        BugBotFace f;
        if (!bugbot_shim_face_get(i, &f)) continue;
        /* Each face: [x1, y1, x2, y2, score, [kp0..kp9]] — 6 elements. */
        py_newlistn(py_list_getitem(outer, i), 6);
        py_Ref face = py_list_getitem(outer, i);
        py_newint  (py_list_getitem(face, 0), (py_i64)f.x1);
        py_newint  (py_list_getitem(face, 1), (py_i64)f.y1);
        py_newint  (py_list_getitem(face, 2), (py_i64)f.x2);
        py_newint  (py_list_getitem(face, 3), (py_i64)f.y2);
        py_newfloat(py_list_getitem(face, 4), (py_f64)f.score);
        py_newlistn(py_list_getitem(face, 5), 10);
        py_Ref kps = py_list_getitem(face, 5);
        for (int k = 0; k < 10; k++) {
            py_newint(py_list_getitem(kps, k), (py_i64)f.kp[k]);
        }
    }
    return true;
}

/* ── go(): inject all robot functions into __main__ globals ──────────────── */

static bool bb_go(int argc, py_StackRef argv) {
    (void)argc; (void)argv;
    py_GlobalRef m = py_getmodule("__main__");
    if (!m) { py_newnone(py_retval()); return true; }

    py_bindfunc(m, "forward",    bb_forward);
    py_bindfunc(m, "backward",   bb_backward);
    py_bindfunc(m, "left",       bb_left);
    py_bindfunc(m, "right",      bb_right);
    py_bindfunc(m, "stop",       bb_stop);
    py_bindfunc(m, "turn",       bb_turn);
    py_bindfunc(m, "spin_right", bb_spin_right);
    py_bindfunc(m, "spin_left",  bb_spin_left);
    py_bindfunc(m, "wait",       bb_wait);
    py_bindfunc(m, "distance",   bb_distance);
    py_bindfunc(m, "lidar_grid", bb_lidar_grid);
    py_bindfunc(m, "heading",    bb_heading);
    py_bindfunc(m, "position",   bb_position);
    py_bindfunc(m, "battery",    bb_battery);
    py_bindfunc(m, "apriltags",      bb_apriltags);
    py_bindfunc(m, "reset_heading",  bb_reset_heading);
    py_bindfunc(m, "drive",          bb_drive);
    py_bindfunc(m, "led",            bb_led);
    py_bindfunc(m, "beep",           bb_beep);
    py_bindfunc(m, "servo",          bb_servo);
    py_bindfunc(m, "camera_suspend", bb_camera_suspend);
    py_bindfunc(m, "camera_resume",  bb_camera_resume);
    py_bindfunc(m, "set_cv",         bb_set_cv);
    py_bindfunc(m, "blobs",          bb_blobs);
    py_bindfunc(m, "edges",          bb_edges);
    py_bindfunc(m, "tinyml_result",  bb_tinyml_result);
    py_bindfunc(m, "faces",          bb_faces);
    py_bindfunc(m, "face_frames",    bb_face_frames);
    py_bindfunc(m, "face_loaded",    bb_face_loaded);
    py_bindfunc(m, "motor_ok",       bb_motor_ok);
    py_bindfunc(m, "motor_raw_test", bb_motor_raw_test);

    py_newnone(py_retval());
    return true;
}

/* ── module registration ──────────────────────────────────────────────────── */

void bugbot_module_init(void) {
    py_GlobalRef mod = py_newmodule("bugbot");

    py_bindfunc(mod, "forward",    bb_forward);
    py_bindfunc(mod, "backward",   bb_backward);
    py_bindfunc(mod, "left",       bb_left);
    py_bindfunc(mod, "right",      bb_right);
    py_bindfunc(mod, "stop",       bb_stop);
    py_bindfunc(mod, "turn",       bb_turn);
    py_bindfunc(mod, "spin_right", bb_spin_right);
    py_bindfunc(mod, "spin_left",  bb_spin_left);
    py_bindfunc(mod, "wait",       bb_wait);
    py_bindfunc(mod, "distance",   bb_distance);
    py_bindfunc(mod, "lidar_grid", bb_lidar_grid);
    py_bindfunc(mod, "heading",    bb_heading);
    py_bindfunc(mod, "position",   bb_position);
    py_bindfunc(mod, "battery",    bb_battery);
    py_bindfunc(mod, "apriltags",      bb_apriltags);
    py_bindfunc(mod, "reset_heading",  bb_reset_heading);
    py_bindfunc(mod, "drive",          bb_drive);
    py_bindfunc(mod, "led",            bb_led);
    py_bindfunc(mod, "beep",           bb_beep);
    py_bindfunc(mod, "servo",          bb_servo);
    py_bindfunc(mod, "camera_suspend", bb_camera_suspend);
    py_bindfunc(mod, "camera_resume",  bb_camera_resume);
    py_bindfunc(mod, "set_cv",         bb_set_cv);
    py_bindfunc(mod, "blobs",          bb_blobs);
    py_bindfunc(mod, "edges",          bb_edges);
    py_bindfunc(mod, "tinyml_result",  bb_tinyml_result);
    py_bindfunc(mod, "faces",          bb_faces);
    py_bindfunc(mod, "face_frames",    bb_face_frames);
    py_bindfunc(mod, "face_loaded",    bb_face_loaded);
    py_bindfunc(mod, "motor_ok",       bb_motor_ok);
    py_bindfunc(mod, "motor_raw_test", bb_motor_raw_test);
    py_bindfunc(mod, "go",             bb_go);
}
