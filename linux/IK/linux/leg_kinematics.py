"""Analytic FK/IK for Ben's single 3R FOC leg.

Coordinate convention (hip frame, metres): +X forward, +Z upward.
At q=[0, 0, 0] every link points vertically down.  ``foot_pitch=0``
also means that the sole link points down.  Angles supplied to this module are
radians; user-facing helper functions are provided for degrees.

This is intentionally offline-only.  It does not command the H723.  The sign
mapping below was confirmed with 5 degree, single-joint H723 tests on the
all-vertical ZERO pose.  Keep H723 motor torque signs separate from this
kinematic target mapping.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import acos, atan2, cos, degrees, hypot, isclose, pi, radians, sin
from typing import NamedTuple


class UnreachableTarget(ValueError):
    """Raised when a Cartesian target is outside the two-link workspace."""


class Point(NamedTuple):
    x: float
    z: float


@dataclass(frozen=True)
class LegGeometry:
    """Planar centre-to-centre dimensions in metres."""

    hip_to_knee_m: float = 0.100
    knee_to_ankle_m: float = 0.100
    ankle_to_sole_m: float = 0.065

    # These are not used by planar FK/IK, but are retained for later URDF.
    leg_width_m: float = 0.070
    hip_fixture_offset_m: float = 0.060


GEOMETRY = LegGeometry()

# Confirmed mapping from the mathematical leg convention to H723 q order
# [hip, knee, ankle].  With +X forward and the all-vertical ZERO pose:
#   H723 hip +    -> thigh forward       (same as mathematical hip +)
#   H723 knee -   -> shank forward       (opposite mathematical knee +)
#   H723 ankle +  -> toe/forefoot up     (same as mathematical ankle +)
# The H723's motor-torque signs are a separate calibration and must not be
# changed as a consequence of this mapping.
KINEMATIC_TO_H723_SIGN = (1.0, -1.0, 1.0)

# This is the actual folding side: knee-forward / orange branch in the first
# plot.  The other solution is mathematically valid but is not this mechanism's
# normal bending configuration.
PHYSICAL_KNEE_BRANCH = -1

# H723 per-joint soft limits, relative to all-vertical ZERO.  The vertical,
# foot-level test needs knee +66.2 degrees while hip/ankle remain below +34.
# Keep the broader mechanical-stop margin exclusively in the H723 hard limits.
H723_SOFT_LIMIT_DEG = (60.0, 70.0, 60.0)  # [hip, knee, ankle]


class LegPose(NamedTuple):
    hip: Point
    knee: Point
    ankle: Point
    sole: Point
    foot_pitch_rad: float


def _absolute_link_angles(q_rad: tuple[float, float, float]) -> tuple[float, float, float]:
    """Return link angles measured CCW from +X."""
    q0, q1, q2 = q_rad
    theta0 = -pi / 2.0 + q0
    theta1 = theta0 + q1
    theta2 = theta1 + q2
    return theta0, theta1, theta2


def fk(q_rad: tuple[float, float, float], geometry: LegGeometry = GEOMETRY) -> LegPose:
    """Forward kinematics for HIP -> KNEE -> ANKLE -> sole contact."""
    t0, t1, t2 = _absolute_link_angles(q_rad)
    hip = Point(0.0, 0.0)
    knee = Point(
        geometry.hip_to_knee_m * cos(t0),
        geometry.hip_to_knee_m * sin(t0),
    )
    ankle = Point(
        knee.x + geometry.knee_to_ankle_m * cos(t1),
        knee.z + geometry.knee_to_ankle_m * sin(t1),
    )
    sole = Point(
        ankle.x + geometry.ankle_to_sole_m * cos(t2),
        ankle.z + geometry.ankle_to_sole_m * sin(t2),
    )
    return LegPose(hip, knee, ankle, sole, foot_pitch_rad=q_rad[0] + q_rad[1] + q_rad[2])


def ik(
    sole_x_m: float,
    sole_z_m: float,
    foot_pitch_rad: float = 0.0,
    knee_branch: int = PHYSICAL_KNEE_BRANCH,
    geometry: LegGeometry = GEOMETRY,
) -> tuple[float, float, float]:
    """Solve analytic planar IK for a desired sole point and sole pitch.

    ``knee_branch`` is +1 or -1 and selects the two geometric elbow/knee
    solutions.  Select the branch that matches the actual leg after plotting;
    it is a kinematic choice, not the H723 motor torque sign.
    """
    if knee_branch not in (-1, 1):
        raise ValueError("knee_branch must be +1 or -1")

    l1 = geometry.hip_to_knee_m
    l2 = geometry.knee_to_ankle_m
    l3 = geometry.ankle_to_sole_m

    # The final link is vertical-down at foot_pitch=0.
    theta2 = -pi / 2.0 + foot_pitch_rad
    wrist_x = sole_x_m - l3 * cos(theta2)
    wrist_z = sole_z_m - l3 * sin(theta2)
    radius = hypot(wrist_x, wrist_z)

    min_radius = abs(l1 - l2)
    max_radius = l1 + l2
    tolerance = 1e-9
    if radius < min_radius - tolerance or radius > max_radius + tolerance:
        raise UnreachableTarget(
            f"wrist radius {radius:.4f} m is outside [{min_radius:.4f}, {max_radius:.4f}] m"
        )

    c_q1 = (radius * radius - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    c_q1 = min(1.0, max(-1.0, c_q1))
    q1 = knee_branch * acos(c_q1)
    theta0 = atan2(wrist_z, wrist_x) - atan2(l2 * sin(q1), l1 + l2 * cos(q1))
    q0 = theta0 + pi / 2.0
    q2 = foot_pitch_rad - q0 - q1
    return q0, q1, q2


def ik_deg(
    sole_x_mm: float,
    sole_z_mm: float,
    foot_pitch_deg: float = 0.0,
    knee_branch: int = PHYSICAL_KNEE_BRANCH,
    geometry: LegGeometry = GEOMETRY,
) -> tuple[float, float, float]:
    """Degree/mm convenience wrapper around :func:`ik`."""
    q = ik(sole_x_mm / 1000.0, sole_z_mm / 1000.0, radians(foot_pitch_deg), knee_branch, geometry)
    return tuple(degrees(value) for value in q)


def controller_from_kinematic_deg(q_ik_deg: tuple[float, float, float]) -> tuple[float, float, float]:
    """Map mathematical angles to confirmed H723 SET [hip, knee, ankle]."""
    return tuple(sign * angle for sign, angle in zip(KINEMATIC_TO_H723_SIGN, q_ik_deg, strict=True))


def within_h723_soft_limit(q_controller_deg: tuple[float, float, float]) -> bool:
    return all(
        abs(angle) <= limit
        for angle, limit in zip(q_controller_deg, H723_SOFT_LIMIT_DEG, strict=True)
    )


def _self_test() -> None:
    neutral = fk((0.0, 0.0, 0.0))
    assert isclose(neutral.sole.x, 0.0, abs_tol=1e-12)
    assert isclose(neutral.sole.z, -0.265, abs_tol=1e-12)

    # A non-singular target validates IK -> FK closure for both branches.
    for branch in (-1, 1):
        q = ik(0.030, -0.220, radians(5.0), branch)
        pose = fk(q)
        assert isclose(pose.sole.x, 0.030, abs_tol=1e-9)
        assert isclose(pose.sole.z, -0.220, abs_tol=1e-9)
        assert isclose(pose.foot_pitch_rad, radians(5.0), abs_tol=1e-9)

    print("FK/IK self-test passed")
    print("neutral sole: x=0.0 mm, z=-265.0 mm")


if __name__ == "__main__":
    _self_test()
