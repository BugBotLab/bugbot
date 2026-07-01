# bugbot.py — MicroPython module that runs on the robot.
# Frozen into firmware or uploaded to LittleFS root.
#
# Wraps the _bugbot C extension (bugbot_module.c) which calls into
# the existing C++ FreeRTOS services via extern "C" shims.
#
# Student-facing API (star-importable, matches the PC-side docs):
#   go()                 — no-op on robot (magic line does nothing here)
#   forward(speed)       — drive forward at speed 0-100
#   backward(speed)      — drive backward
#   left(speed)          — strafe left  (mecanum)
#   right(speed)         — strafe right (mecanum)
#   turn(deg)            — turn in place by deg degrees (blocking)
#   stop()               — all stop
#   wait(seconds)        — time.sleep alias
#   led(r_or_name,g,b)   — set RGB LED
#   beep(freq, ms=200)   — buzzer
#   servo(index, angle)  — servo position
#   distance()           — nearest LiDAR return (mm)
#   heading()            — yaw in degrees (0-360)
#   position()           — (x_mm, y_mm) from EKF
#   battery()            — battery level 0-100

import time

# _bugbot is the C extension module compiled into the MicroPython firmware image.
# If it doesn't exist yet (early dev / simulator), the stubs below fall back to
# print statements so student scripts can still be tested.
try:
    import _bugbot as _hw
    _HAVE_HW = True
except ImportError:
    _HAVE_HW = False

# ── Direction constants (match lib/core/DriveDefs.hpp) ─────────────────────────
_STOP      = 0
_FORWARD   = 1
_BACKWARD  = 2
_STRAFE_L  = 3
_STRAFE_R  = 4
_TURN_L    = 5
_TURN_R    = 6

# Approximate calibration constants (match AppConfig defaults).
# Used for blocking time-based moves when the firmware doesn't self-terminate.
_CM_PER_SEC_AT_100 = 20.0
_DEG_PER_SEC_TURN  = 90.0
_TURN_SPEED        = 40.0

# ── Named LED colours ──────────────────────────────────────────────────────────
_COLOURS = {
    "red":     (255, 0, 0),   "green":   (0, 255, 0),   "blue":  (0, 0, 255),
    "yellow":  (255, 255, 0), "cyan":    (0, 255, 255),  "magenta": (255, 0, 255),
    "white":   (255, 255, 255), "black": (0, 0, 0),      "off":   (0, 0, 0),
    "orange":  (255, 128, 0), "purple":  (128, 0, 128),  "pink":  (255, 105, 180),
}


# ── Magic ──────────────────────────────────────────────────────────────────────
def go():
    """No-op on the robot — but injects all public names into __main__ so the
    student can call forward(), stop() etc. without 'from bugbot import *'."""
    try:
        import __main__ as _m
        _g = vars(_m)
        for _n in __all__:
            if _n != 'go':
                _g[_n] = globals()[_n]
    except Exception:
        pass


# ── Motion ─────────────────────────────────────────────────────────────────────
def forward(speed, distance=None):
    """Drive forward. speed = 0-100. distance in cm blocks until reached."""
    speed = max(0, min(100, float(speed)))
    if _HAVE_HW:
        _hw.drive(_FORWARD, speed)
    else:
        print(f"[sim] forward({speed})")
    if distance is not None:
        secs = abs(distance) / (_CM_PER_SEC_AT_100 * speed / 100.0) if speed else 0
        time.sleep(secs)
        stop()


def backward(speed, distance=None):
    speed = max(0, min(100, float(speed)))
    if _HAVE_HW:
        _hw.drive(_BACKWARD, speed)
    else:
        print(f"[sim] backward({speed})")
    if distance is not None:
        secs = abs(distance) / (_CM_PER_SEC_AT_100 * speed / 100.0) if speed else 0
        time.sleep(secs)
        stop()


def left(speed):
    """Strafe left (mecanum drive)."""
    speed = max(0, min(100, float(speed)))
    if _HAVE_HW:
        _hw.drive(_STRAFE_L, speed)
    else:
        print(f"[sim] left({speed})")


def right(speed):
    """Strafe right (mecanum drive)."""
    speed = max(0, min(100, float(speed)))
    if _HAVE_HW:
        _hw.drive(_STRAFE_R, speed)
    else:
        print(f"[sim] right({speed})")


def turn(deg):
    """Turn in place by deg degrees (positive = clockwise). Blocking."""
    direction = _TURN_R if deg >= 0 else _TURN_L
    if _HAVE_HW:
        _hw.drive(direction, _TURN_SPEED)
    else:
        print(f"[sim] turn({deg})")
    time.sleep(abs(deg) / _DEG_PER_SEC_TURN)
    stop()


def stop():
    if _HAVE_HW:
        _hw.stop()
    else:
        print("[sim] stop()")


def wait(seconds):
    time.sleep(seconds)


# ── Output ─────────────────────────────────────────────────────────────────────
def led(r, g=None, b=None):
    """Set RGB LED. led("red") or led(255, 0, 0)."""
    if isinstance(r, str):
        r, g, b = _COLOURS.get(r.lower(), (0, 0, 0))
    r = int(max(0, min(255, r)))
    g = int(max(0, min(255, g)))
    b = int(max(0, min(255, b)))
    if _HAVE_HW:
        _hw.led(r, g, b)
    else:
        print(f"[sim] led({r},{g},{b})")


def beep(freq, ms=200):
    if _HAVE_HW:
        _hw.beep(freq)
        time.sleep(ms / 1000.0)
        _hw.beep(0)
    else:
        print(f"[sim] beep({freq}, {ms}ms)")


def servo(index, angle):
    if _HAVE_HW:
        _hw.servo(int(index), float(angle))
    else:
        print(f"[sim] servo({index}, {angle})")


# ── Sensors ────────────────────────────────────────────────────────────────────
def distance():
    """Return closest LiDAR reading in mm."""
    if _HAVE_HW:
        return _hw.distance()
    print("[sim] distance() -> 500")
    return 500.0


def heading():
    """Return yaw in degrees (0-360, clockwise from north)."""
    if _HAVE_HW:
        return _hw.heading()
    print("[sim] heading() -> 0")
    return 0.0


def position():
    """Return (x_mm, y_mm) from the EKF pose estimate."""
    if _HAVE_HW:
        return _hw.position()
    print("[sim] position() -> (0, 0)")
    return (0.0, 0.0)


def battery():
    """Return battery level 0-100."""
    if _HAVE_HW:
        return _hw.battery()
    print("[sim] battery() -> 100")
    return 100


# ── Star-import list ───────────────────────────────────────────────────────────
# Students can write `from bugbot import *` to get all of these without the
# `bugbot.` prefix.
__all__ = [
    "go",
    "forward", "backward", "left", "right", "turn", "stop", "wait",
    "led", "beep", "servo",
    "distance", "heading", "position", "battery",
]
