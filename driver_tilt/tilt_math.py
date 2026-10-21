"""Pure tilt math: sensor axes -> van axes -> roll/pitch degrees.

No I/O. Van frame convention: +x forward, +y left, +z up. Roll is rotation
about x (positive = left side down), pitch is rotation about y (positive =
nose up). At rest the accelerometer reads +1 g along the van's +z (up),
so a level van yields roll = pitch = 0.

Roll/pitch come from the accelerometer alone: a parked/driving van is
quasi-static for inclination purposes, and a gyro would add bias drift
without improving a level indicator.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

Vec3 = tuple[float, float, float]

_SENSOR_AXES = {"x": 0, "y": 1, "z": 2}


@dataclass(frozen=True)
class AxisMap:
    """Maps sensor axes to van axes.

    Each field is one of "+x", "-x", "+y", "-y", "+z", "-z": which sensor
    axis (with sign) supplies that van axis. The three fields must use
    three distinct sensor axes (a valid orientation, not a degenerate one).
    """

    x: str = "+x"
    y: str = "+y"
    z: str = "+z"

    def __post_init__(self) -> None:
        picked = []
        for van_axis, spec in (("x", self.x), ("y", self.y), ("z", self.z)):
            if len(spec) != 2 or spec[0] not in "+-" or spec[1] not in _SENSOR_AXES:
                raise ValueError(f"axis_{van_axis} must be one of +x,-x,+y,-y,+z,-z; got {spec!r}")
            picked.append(spec[1])
        if len(set(picked)) != 3:
            raise ValueError(
                f"axis mapping must use each sensor axis exactly once; got "
                f"x={self.x!r} y={self.y!r} z={self.z!r}"
            )


def remap_axes(accel: Vec3, axis_map: AxisMap) -> Vec3:
    """Rotate a raw sensor-frame acceleration vector into the van frame."""

    def pick(spec: str) -> float:
        value = accel[_SENSOR_AXES[spec[1]]]
        return -value if spec[0] == "-" else value

    return (pick(axis_map.x), pick(axis_map.y), pick(axis_map.z))


def roll_pitch_deg(van_accel: Vec3) -> tuple[float, float]:
    """Roll and pitch in degrees from a van-frame acceleration vector.

    Raises ValueError on a zero vector (no gravity reference, e.g. a
    zeroed/faulted read) -- the caller should treat that as a sensor error.
    """
    ax, ay, az = van_accel
    if ax == 0.0 and ay == 0.0 and az == 0.0:
        raise ValueError("zero acceleration vector: no gravity reference")
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
    return roll, pitch


def plausible_gravity(accel: Vec3, min_g: float, max_g: float) -> bool:
    """True if |accel| (in g) lies within [min_g, max_g].

    A cornering/braking/bumpy-road sample deviates from 1 g; its angle
    estimate is wrong, so the driver skips smoothing updates for it. The
    band is loose by default because an uncalibrated MPU-6050 is commonly
    5-15% off in scale.
    """
    magnitude = math.sqrt(sum(c * c for c in accel))
    return min_g <= magnitude <= max_g


def ema(previous: float | None, sample: float, alpha: float) -> float:
    """Exponential moving average; first sample (previous None) seeds it."""
    if previous is None:
        return sample
    return previous + alpha * (sample - previous)


def apply_offset(value_deg: float, offset_deg: float) -> float:
    """Subtract a level-zero offset."""
    return value_deg - offset_deg
