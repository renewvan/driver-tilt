import math

import pytest

from node_tilt.tilt_math import (
    AxisMap,
    apply_offset,
    ema,
    plausible_gravity,
    remap_axes,
    roll_pitch_deg,
)


def test_level_van_reads_zero() -> None:
    roll, pitch = roll_pitch_deg((0.0, 0.0, 1.0))
    assert roll == pytest.approx(0.0)
    assert pitch == pytest.approx(0.0)


def test_left_side_down_is_positive_roll() -> None:
    # Van frame: +y is left. Tilting left-down puts gravity's reaction on +y.
    g = 1.0
    a = math.radians(5)
    roll, pitch = roll_pitch_deg((0.0, g * math.sin(a), g * math.cos(a)))
    assert roll == pytest.approx(5.0)
    assert pitch == pytest.approx(0.0, abs=1e-9)


def test_nose_up_is_positive_pitch() -> None:
    g = 1.0
    a = math.radians(7)
    # Nose up: reaction on -x (forward axis points up-and-away from gravity).
    roll, pitch = roll_pitch_deg((-g * math.sin(a), 0.0, g * math.cos(a)))
    assert pitch == pytest.approx(7.0)
    assert roll == pytest.approx(0.0, abs=1e-9)


def test_zero_vector_is_rejected() -> None:
    with pytest.raises(ValueError):
        roll_pitch_deg((0.0, 0.0, 0.0))


def test_face_down_measurement_levels_with_flipped_axes() -> None:
    # Measured on the Pi, GY-521 face-down on the bench: gravity on -z.
    raw = (0.0615, -0.0328, -1.1611)
    van = remap_axes(raw, AxisMap(x="+x", y="-y", z="-z"))
    roll, pitch = roll_pitch_deg(van)
    assert abs(roll) < 3.5
    assert abs(pitch) < 3.5
    # Without the mapping the same sample reads ~180 degrees of roll.
    raw_roll, _ = roll_pitch_deg(raw)
    assert abs(raw_roll) > 170


def test_edge_mounted_board_maps_x_to_up() -> None:
    # Measured: board standing on edge, gravity on +x. Map sensor +x -> van up.
    raw = (1.0395, 0.0125, -0.0964)
    van = remap_axes(raw, AxisMap(x="-z", y="+y", z="+x"))
    roll, pitch = roll_pitch_deg(van)
    assert abs(roll) < 2
    assert abs(pitch) < 7  # 0.0964 g residual on the old z axis


@pytest.mark.parametrize(
    "x,y,z",
    [
        ("+x", "+x", "+z"),  # duplicate sensor axis
        ("+x", "+y", "+y"),
        ("x", "+y", "+z"),  # no sign
        ("+w", "+y", "+z"),  # unknown axis
        ("++x", "+y", "+z"),
    ],
)
def test_invalid_axis_map_rejected(x: str, y: str, z: str) -> None:
    with pytest.raises(ValueError):
        AxisMap(x, y, z)


def test_remap_preserves_vector_magnitude() -> None:
    raw = (0.3, -0.4, 0.8)
    van = remap_axes(raw, AxisMap(x="+z", y="-x", z="+y"))
    assert math.sqrt(sum(c * c for c in van)) == pytest.approx(math.sqrt(sum(c * c for c in raw)))


def test_plausible_gravity_band() -> None:
    assert plausible_gravity((0.0, 0.0, 1.0), 0.8, 1.25)
    assert plausible_gravity((0.0, 0.0, 1.16), 0.8, 1.25)  # measured scale error
    assert not plausible_gravity((0.5, 0.0, 1.2), 0.8, 1.25)  # hard braking
    assert not plausible_gravity((0.0, 0.0, 0.3), 0.8, 1.25)  # free-fall-ish


def test_ema_seeds_then_converges() -> None:
    value = ema(None, 10.0, 0.2)
    assert value == 10.0
    for _ in range(60):
        value = ema(value, 0.0, 0.2)
    assert value == pytest.approx(0.0, abs=1e-4)


def test_apply_offset() -> None:
    assert apply_offset(5.5, 5.0) == pytest.approx(0.5)
