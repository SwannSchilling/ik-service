"""arm7 — the 7-DOF POC arm as a pure-Python reference implementation.

Ported 1:1 from the libpick_ik_core C++ reference FK (tests/arm7_fk.hpp and
examples/arm7_cross_check/main.cpp) and the p5.js POC's computeURDFFK:

    base_link at identity;
    for each joint (document order):
        child frame = parent frame * joint origin transform * R(axis, q)
    tool0 = link7 + 0.126 m along link7 z (fixed joint).

Units: meters, radians. The p5 display SCALE factor is NOT applied.

Roles:
  1. FK callback + robot factory for the FastAPI service (the solvers are C++).
  2. Third independent FK implementation: tests pin it against the verified
     anchor poses in ARM7_KINEMATIC_SPEC.md section 5.
"""
from __future__ import annotations

import math

import numpy as np

import pickik

# ----------------------------------------------------------------------------
# Joint table — identical constants to the C++ reference.
# Row: (origin_z [m], origin roll (rpy about x) [rad], min, max, max_velocity)
# J2/J4/J6: -90 deg; J3/J5/J7: +90 deg. All joint axes are local z.
# ----------------------------------------------------------------------------
_PI = math.pi

JOINTS = [
    (0.000, 0.0, -_PI, _PI, 2.17),       # J1 base yaw
    (0.340, -_PI / 2, -2.09, 2.09, 2.17),  # J2 shoulder pitch
    (0.000, _PI / 2, -_PI, _PI, 2.17),    # J3 shoulder roll
    (0.400, -_PI / 2, -2.09, 2.09, 2.17),  # J4 elbow pitch
    (0.000, _PI / 2, -_PI, _PI, 2.61),    # J5 forearm roll
    (0.400, -_PI / 2, -2.09, 2.09, 2.61),  # J6 wrist pitch
    (0.000, _PI / 2, -_PI, _PI, 2.61),    # J7 tool roll
]
TOOL_OFFSET = 0.126  # m, link7 -> tool0 (fixed joint)

# The p5 POC's "all zero" slider state: sliders snap to 0.01 rad steps anchored
# at each joint's lower limit, so filling 0 yields -0.00159265 on the
# pi-limited joints and 0.0 on the 2.09-limited ones.
QUANTIZED_ZERO_SEED = [
    -0.00159265, 0.0, -0.00159265, 0.0, -0.00159265, 0.0, -0.00159265,
]

LOCAL_AXES = [[0.0, 0.0, 1.0]] * 7


def _rot_x(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rot_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# Precompute the joint origin transforms (constant, like the C++ reference).
_JOINT_ORIGINS: list[np.ndarray] = []
for _oz, _roll, *_ in JOINTS:
    _m = np.eye(4)
    if _roll != 0.0:
        _m[:3, :3] = _rot_x(_roll)
    _m[2, 3] = _oz
    _JOINT_ORIGINS.append(_m)


def link_frames(q: list[float]) -> list[np.ndarray]:
    """All joint child frames + tool0, as 4x4 matrices (base frame, meters).

    frames[i] for 0 <= i < 7: frame of joint i's child link (pivot frame,
    including origin and current joint rotation); frames[7]: tool0.
    """
    if len(q) != 7:
        raise ValueError(f"q must have 7 joints, got {len(q)}")
    frames: list[np.ndarray] = []
    frame = np.eye(4)
    for i in range(7):
        frame = frame @ _JOINT_ORIGINS[i]
        rot = np.eye(4)
        rot[:3, :3] = _rot_z(q[i])
        frame = frame @ rot
        frames.append(frame.copy())
    # tool0 = link7 + TOOL_OFFSET along the LINK7 (frame) z-axis — a right
    # multiplication, exactly like the C++ reference (frame * Translation3d).
    offset = np.eye(4)
    offset[2, 3] = TOOL_OFFSET
    frames.append(frame @ offset)
    return frames


def tool0_pose(q: list[float]) -> np.ndarray:
    """tool0 frame in the base frame (4x4)."""
    return link_frames(q)[-1]


def fk_callback(q: list[float]) -> list[np.ndarray]:
    """FK callback for the pickik binding: q -> n+1 4x4 frames."""
    return link_frames(q)


ROBOT: pickik.Robot = pickik.make_robot(
    [
        pickik.JointSpec(min_val, max_val, max_velocity=max_vel)
        for _oz, _pitch, min_val, max_val, max_vel in JOINTS
    ]
)
