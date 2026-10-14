import struct
import sys
import types

import pytest

from node_tilt import imu as imu_module
from node_tilt.imu import MPU6050

GRAVITY_RAW = 16384  # 1 g on the +/-2 g range


class FakeChip:
    """An MPU-6050 that can fall back to its power-on sleep state, like a brownout does.

    While asleep every accelerometer register reads zero; writing PWR_MGMT_1 = 0 wakes it.
    """

    def __init__(self, asleep_reads_after_wake: int = 0) -> None:
        self.asleep = False
        self.wake_count = 0
        # How many reads after a wake still return zeros (a chip that keeps resetting).
        self._stubborn = asleep_reads_after_wake
        self.closed = False

    def read_byte_data(self, _addr: int, reg: int) -> int:
        assert reg == 0x75
        return 0x68

    def write_byte_data(self, _addr: int, reg: int, value: int) -> None:
        if reg == 0x6B and value == 0x00:
            self.asleep = False
            self.wake_count += 1

    def read_i2c_block_data(self, _addr: int, _reg: int, length: int) -> list[int]:
        assert length == 6
        if self.asleep or self._stubborn > 0:
            if not self.asleep:
                self._stubborn -= 1
            return [0] * 6
        return list(struct.pack(">3h", 0, 0, GRAVITY_RAW))

    def close(self) -> None:
        self.closed = True


@pytest.fixture
def chip(monkeypatch: pytest.MonkeyPatch) -> FakeChip:
    fake = FakeChip()
    monkeypatch.setitem(sys.modules, "smbus2", types.SimpleNamespace(SMBus=lambda _n: fake))
    monkeypatch.setattr(imu_module, "_WAKE_SETTLE_S", 0.0)
    return fake


def test_reads_gravity_while_the_chip_is_awake(chip: FakeChip) -> None:
    sensor = MPU6050(bus_number=1, address=0x68)
    assert sensor.read_accel_g() == (0.0, 0.0, 1.0)
    assert chip.wake_count == 1  # only the wake from __post_init__


def test_rewakes_and_returns_a_real_sample_after_the_chip_resets_to_sleep(
    chip: FakeChip,
) -> None:
    sensor = MPU6050(bus_number=1, address=0x68)
    chip.asleep = True  # supply dip: the chip is back in its power-on state

    assert sensor.read_accel_g() == (0.0, 0.0, 1.0)
    assert chip.wake_count == 2
    # Awake again, so the next read needs no further wake.
    assert sensor.read_accel_g() == (0.0, 0.0, 1.0)
    assert chip.wake_count == 2


def test_a_chip_that_stays_zero_after_the_rewake_is_returned_as_zero(chip: FakeChip) -> None:
    sensor = MPU6050(bus_number=1, address=0x68)
    chip._stubborn = 2  # the re-wake read is still zero: a real fault, not a one-off reset

    assert sensor.read_accel_g() == (0.0, 0.0, 0.0)  # the caller counts this as an error
    assert chip.wake_count == 2  # woke once, did not loop


def test_unexpected_who_am_i_is_still_rejected(chip: FakeChip) -> None:
    chip.read_byte_data = lambda _a, _r: 0x70  # type: ignore[method-assign]
    with pytest.raises(OSError, match="WHO_AM_I"):
        MPU6050(bus_number=1, address=0x68)
    assert chip.closed
