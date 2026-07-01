"""BugBot X-omni drive mixing.

Pure NumPy, no Isaac dependency, so it can be unit-tested on its own and reused
by the Isaac Lab env, the ROS 2 bridge, and any analysis. The matrix below is the
authoritative Python copy of the firmware mix in
``firmware/main/lib/drivers/MotionLib.cpp`` and MUST stay identical to it.

Motor order is (BR, FR, BL, FL). A body twist is (longitudinal, lateral, rot):
longitudinal = forward +, lateral = right +, rot = CCW / left +.

Firmware basis vectors:
    forward      = [ +, -, +, - ]
    strafe right = [ +, +, -, - ]
    turn left    = [ +, -, -, + ]
"""

from __future__ import annotations

import numpy as np

# Wheel command = MIX @ [longitudinal, lateral, rot], rows ordered (BR, FR, BL, FL).
MIX = np.array(
    [
        [+1.0, +1.0, +1.0],  # BR = +lon + lat + rot
        [-1.0, +1.0, -1.0],  # FR = -lon + lat - rot
        [+1.0, -1.0, -1.0],  # BL = +lon - lat - rot
        [-1.0, -1.0, +1.0],  # FL = -lon - lat + rot
    ],
    dtype=np.float64,
)

# Forward map (odometry): twist = UNMIX @ wheels. MIX has full column rank 3, so
# its pseudo-inverse recovers the twist exactly from consistent wheel commands.
UNMIX = np.linalg.pinv(MIX)

WHEEL_ORDER = ("BR", "FR", "BL", "FL")


def twist_to_wheels(longitudinal: float, lateral: float, rot: float) -> np.ndarray:
    """Map a body twist to the four wheel commands (BR, FR, BL, FL)."""
    return MIX @ np.array([longitudinal, lateral, rot], dtype=np.float64)


def wheels_to_twist(wheels) -> np.ndarray:
    """Recover the body twist (longitudinal, lateral, rot) from wheel commands."""
    return UNMIX @ np.asarray(wheels, dtype=np.float64)


def normalise_wheels(wheels, max_abs: float = 1.0):
    """Scale wheel commands so the largest magnitude fits within ``max_abs``,
    matching the firmware's ``drive()`` capping behaviour. Returns the scaled
    wheels; commands already within range are returned unchanged."""
    wheels = np.asarray(wheels, dtype=np.float64)
    peak = float(np.max(np.abs(wheels)))
    if peak <= max_abs or peak == 0.0:
        return wheels
    return wheels * (max_abs / peak)
