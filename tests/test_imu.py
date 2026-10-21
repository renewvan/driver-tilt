import struct
import sys
import types

import pytest

from driver_tilt import imu as imu_module
from driver_tilt.imu import MPU6050

GRAVITY_RAW = 16384  # 1 g on the +/-2 g range
SLEEP_BIT = 0x40


class FakeChip:
    """An MPU-6050 that can fall back to its power-on sleep state, like a brownout does.

    Two failure modes, both seen on the van's Pi:
    - `reset()`: a full reset. PWR_MGMT_1 reads 0x40 and every accelerometer register reads 0.
    - `sleep()`: the SLEEP bit set on a running chip. Reads keep returning the last sample,
      frozen, so a value check alone cannot tell it from a healthy sensor.
    Writing PWR_MGMT_1 = 0 wakes it either way.
    """

    def __init__(self) -> None:
        self.pwr_mgmt_1 = 0x00
        self.sample = (0, 0, GRAVITY_RAW)
        self.wake_count = 0
        self.zero_reads_after_wake = 0  # a chip that is still zero right after waking
        self.closed = False

    def reset(self) -> None:
        self.pwr_mgmt_1 = SLEEP_BIT
        self.sample = (0, 0, 0)

    def sleep(self) -> None:
        self.pwr_mgmt_1 |= SLEEP_BIT

    def read_byte_data(self, _addr: int, reg: int) -> int:
        return 0x68 if reg == 0x75 else self.pwr_mgmt_1

    def write_byte_data(self, _addr: int, reg: int, value: int) -> None:
        if reg == 0x6B and value == 0x00:
            self.pwr_mgmt_1 = 0x00
            self.wake_count += 1
            if self.sample == (0, 0, 0) and not self.zero_reads_after_wake:
                self.sample = (0, 0, GRAVITY_RAW)  # a reset chip comes back reading gravity

    def read_i2c_block_data(self, _addr: int, _reg: int, length: int) -> list[int]:
        assert length == 6
        if self.zero_reads_after_wake > 0:
            self.zero_reads_after_wake -= 1
            return [0] * 6
        return list(struct.pack(">3h", *self.sample))

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


def test_rewakes_after_a_full_reset_reads_zeros(chip: FakeChip) -> None:
    sensor = MPU6050(bus_number=1, address=0x68)
    chip.reset()

    assert sensor.read_accel_g() == (0.0, 0.0, 1.0)
    assert chip.wake_count == 2
    # Awake again, so the next read needs no further wake.
    assert sensor.read_accel_g() == (0.0, 0.0, 1.0)
    assert chip.wake_count == 2


def test_rewakes_when_the_chip_sleeps_with_frozen_registers(chip: FakeChip) -> None:
    # The frozen sample is non-zero, so only the SLEEP bit gives it away.
    sensor = MPU6050(bus_number=1, address=0x68)
    chip.sample = (-1108, 15308, 2928)
    chip.sleep()

    assert sensor.read_accel_g() == (-1108 / 16384, 15308 / 16384, 2928 / 16384)
    assert chip.wake_count == 2
    assert chip.pwr_mgmt_1 == 0x00


def test_a_chip_that_stays_zero_after_the_rewake_is_returned_as_zero(chip: FakeChip) -> None:
    sensor = MPU6050(bus_number=1, address=0x68)
    chip.reset()
    chip.zero_reads_after_wake = 3  # zero through every wake: a real fault, not a one-off reset

    assert sensor.read_accel_g() == (0.0, 0.0, 0.0)  # the caller counts this as an error
    # Startup wake, one for the SLEEP bit, one for the zero sample: bounded, never a loop.
    assert chip.wake_count == 3


def test_unexpected_who_am_i_is_still_rejected(chip: FakeChip) -> None:
    chip.read_byte_data = lambda _a, _r: 0x70  # type: ignore[method-assign]
    with pytest.raises(OSError, match="WHO_AM_I"):
        MPU6050(bus_number=1, address=0x68)
    assert chip.closed
