"""Verify the ROS-side X-omni mix matches the firmware basis vectors.
Pure NumPy, runnable without ROS:

    python ros2/bugbot_ros/test/test_mixing_parity.py
"""

import os
import sys

import numpy as np

# Import mixing.py directly (avoid needing the installed ROS package).
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "bugbot_ros"))
from mixing import twist_to_wheels, wheels_to_twist  # noqa: E402


def test_basis_matches_firmware():
    np.testing.assert_allclose(twist_to_wheels(1, 0, 0), [+1, -1, +1, -1])  # forward
    np.testing.assert_allclose(twist_to_wheels(0, 1, 0), [+1, +1, -1, -1])  # strafe right
    np.testing.assert_allclose(twist_to_wheels(0, 0, 1), [+1, -1, -1, +1])  # turn left


def test_roundtrip():
    for tw in [(0.3, -0.2, 0.5), (-1.0, 0.0, 0.0)]:
        np.testing.assert_allclose(wheels_to_twist(twist_to_wheels(*tw)), tw, atol=1e-9)


if __name__ == "__main__":
    test_basis_matches_firmware()
    test_roundtrip()
    print("ros mix parity OK")
