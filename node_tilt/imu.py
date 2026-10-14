"""Minimal MPU-6050 (GY-521) accelerometer reader over I2C.

Talks to the chip directly via smbus2, same approach as node-tank's ADS1115
reader. Only the accelerometer is used (see tilt_math.py).
"""

from __future__ import annotations

import logging
import struct
import time
from dataclasses import dataclass

logger = logging.getLogger(__name__)

# Register map (MPU-6050 register map rev 4.2)
_REG_PWR_MGMT_1 = 0x6B
_REG_ACCEL_CONFIG = 0x1C
_REG_ACCEL_XOUT_H = 0x3B
_REG_WHO_AM_I = 0x75

_WHO_AM_I_EXPECTED = 0x68
_ACCEL_FS_2G = 0x00
_LSB_PER_G_2G = 16384.0
# After clearing SLEEP the accelerometer needs a moment before the first
# valid sample; reading immediately returns all-zero registers (seen on-device).
_WAKE_SETTLE_S = 0.1


@dataclass
class MPU6050:
    bus_number: int
    address: int

    def __post_init__(self) -> None:
        import smbus2  # imported lazily so tests never need real hardware/I2C

        self._bus = smbus2.SMBus(self.bus_number)
        who = self._bus.read_byte_data(self.address, _REG_WHO_AM_I)
        if who != _WHO_AM_I_EXPECTED:
            self._bus.close()
            raise OSError(
                f"Unexpected WHO_AM_I 0x{who:02x} at 0x{self.address:02x} "
                f"(expected 0x{_WHO_AM_I_EXPECTED:02x}; clone chips that "
                "report another ID are not supported)"
            )
        self._wake()

    def _wake(self) -> None:
        """Wake from sleep (clears SLEEP bit, selects the internal 8 MHz oscillator)."""
        self._bus.write_byte_data(self.address, _REG_PWR_MGMT_1, 0x00)
        self._bus.write_byte_data(self.address, _REG_ACCEL_CONFIG, _ACCEL_FS_2G)
        time.sleep(_WAKE_SETTLE_S)

    def _read_raw(self) -> tuple[int, int, int]:
        data = bytes(self._bus.read_i2c_block_data(self.address, _REG_ACCEL_XOUT_H, 6))
        return struct.unpack(">3h", data)

    def read_accel_g(self) -> tuple[float, float, float]:
        """Read the accelerometer in g, sensor frame (+/-2 g range).

        An all-zero sample is not a real reading (a resting sensor always sees ~1 g): it is
        the chip's power-on state, which a supply dip puts it back into. Wake it again and
        read once more so one brownout does not leave the node in `sensor_error` until the
        container restarts. A second zero sample is returned as-is for the caller to count.
        """
        raw = self._read_raw()
        if raw == (0, 0, 0):
            logger.warning("MPU-6050 read all zeros (reset to sleep?); waking it again")
            self._wake()
            raw = self._read_raw()
        ax, ay, az = raw
        return ax / _LSB_PER_G_2G, ay / _LSB_PER_G_2G, az / _LSB_PER_G_2G

    def close(self) -> None:
        self._bus.close()
