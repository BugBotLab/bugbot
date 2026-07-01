"""Tests for the BugBot X-omni mix. Pure NumPy, runnable without Isaac:

    python -m pytest simulation/tests
    # or: python simulation/tests/test_mixing.py
"""

import os
import sys

import numpy as np

# Import mixing.py directly (not via the bugbot_tasks package, whose __init__
# pulls in gymnasium/Isaac). Keeps this test runnable in plain Python.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "bugbot_tasks"))
from mixing import (  # noqa: E402
    MIX,
    twist_to_wheels,
    wheels_to_twist,
    normalise_wheels,
)


def test_basis_vectors_match_firmware():
    # forward = [+, -, +, -]  (BR, FR, BL, FL)
    np.testing.assert_allclose(twist_to_wheels(1, 0, 0), [+1, -1, +1, -1])
    # strafe right = [+, +, -, -]
    np.testing.assert_allclose(twist_to_wheels(0, 1, 0), [+1, +1, -1, -1])
    # turn left = [+, -, -, +]
    np.testing.assert_allclose(twist_to_wheels(0, 0, 1), [+1, -1, -1, +1])


def test_roundtrip_twist_is_recovered():
    for tw in [(0.3, -0.2, 0.5), (-1.0, 0.0, 0.0), (0.1, 0.1, -0.1)]:
        wheels = twist_to_wheels(*tw)
        np.testing.assert_allclose(wheels_to_twist(wheels), tw, atol=1e-9)


def test_mix_has_full_column_rank():
    assert np.linalg.matrix_rank(MIX) == 3


def test_normalise_caps_peak():
    w = twist_to_wheels(1, 1, 1)  # peak magnitude 3
    n = normalise_wheels(w, max_abs=1.0)
    assert np.isclose(np.max(np.abs(n)), 1.0)
    # direction preserved
    np.testing.assert_allclose(n, w / 3.0)


def test_normalise_leaves_small_commands():
    w = twist_to_wheels(0.2, 0.0, 0.0)  # peak 0.2
    np.testing.assert_allclose(normalise_wheels(w, 1.0), w)


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"ok  {name}")
    print("all mixing tests passed")
