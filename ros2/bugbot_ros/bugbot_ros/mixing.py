"""BugBot X-omni drive mixing (ROS-side copy).

Authoritative copy of the drive mix, kept identical to the firmware
(``firmware/main/lib/drivers/MotionLib.cpp``) and the simulation
(``simulation/bugbot_tasks/mixing.py``). Duplicated here so the ROS package
builds standalone with no cross-package import; if you change one, change all.

Motor order (BR, FR, BL, FL). Body twist = (longitudinal, lateral, rot):
longitudinal = forward +, lateral = right +, rot = CCW / left +.
"""

from __future__ import annotations

import numpy as np

MIX = np.array(
    [
        [+1.0, +1.0, +1.0],  # BR = +lon + lat + rot
        [-1.0, +1.0, -1.0],  # FR = -lon + lat - rot
        [+1.0, -1.0, -1.0],  # BL = +lon - lat - rot
        [-1.0, -1.0, +1.0],  # FL = -lon - lat + rot
    ],
    dtype=np.float64,
)
UNMIX = np.linalg.pinv(MIX)
WHEEL_ORDER = ("BR", "FR", "BL", "FL")


def twist_to_wheels(longitudinal: float, lateral: float, rot: float) -> np.ndarray:
    """Body twist -> four wheel commands (BR, FR, BL, FL)."""
    return MIX @ np.array([longitudinal, lateral, rot], dtype=np.float64)


def wheels_to_twist(wheels) -> np.ndarray:
    """Four wheel commands -> body twist (longitudinal, lateral, rot)."""
    return UNMIX @ np.asarray(wheels, dtype=np.float64)


def normalise_wheels(wheels, max_abs: float = 1.0):
    """Scale wheel commands so the peak magnitude fits within ``max_abs`` (matches
    the firmware ``drive()`` cap). Commands already in range are unchanged."""
    wheels = np.asarray(wheels, dtype=np.float64)
    peak = float(np.max(np.abs(wheels)))
    if peak <= max_abs or peak == 0.0:
        return wheels
    return wheels * (max_abs / peak)
