"""Publish loop: MPU-6050 accel read -> axis remap -> roll/pitch ->
renewvan/tilt/<id>/<property>.

Identity field (retained, republished on every MQTT (re)connect by the Publisher):
  driver               this Driver's key ("tilt"), the segment of renewvan/tilt/info

Live fields (retained, published on change beyond a deadband, plus a
heartbeat so a retained value never goes silently stale):
  roll_deg, pitch_deg  level-corrected, smoothed degrees
  status               ok | sensor_error

Command topic (unretained):
  renewvan/tilt/<id>/level_zero/set   capture the current attitude as level.
      Payload "reset" (or "null") clears the stored offset.
The offsets persist in state/<id>.json so a restart keeps the install zero.
"""

from __future__ import annotations

import json
import logging
import time
from dataclasses import dataclass
from pathlib import Path

from driver_tilt.config import DEFAULT_STATE_DIR, AppConfig, DriverConfig
from driver_tilt.imu import MPU6050
from driver_tilt.info import build_info
from driver_tilt.publisher import Publisher
from driver_tilt.tilt_math import (
    AxisMap,
    apply_offset,
    ema,
    plausible_gravity,
    remap_axes,
    roll_pitch_deg,
)

logger = logging.getLogger(__name__)

# Default state directory for machine-written persistence (same convention as
# driver-tank); `state_dir` in config.ini overrides it per install.
_STATE_DIR = DEFAULT_STATE_DIR


@dataclass
class TiltState:
    """Runtime state across reads."""

    roll_smoothed: float | None = None  # raw (pre-offset) smoothed angles
    pitch_smoothed: float | None = None
    roll_offset: float = 0.0
    pitch_offset: float = 0.0
    consecutive_errors: int = 0
    last_status: str | None = None
    last_published_roll: float | None = None
    last_published_pitch: float | None = None
    last_publish_at: float = 0.0


def identity_topics(entity_id: str, driver_key: str) -> dict[str, str]:
    """Retained identity fields of the tilt instance, topic -> payload.

    Tilt has no other static fields; `driver` names the owning Driver so
    consumers can attribute the instance to it (hub's schema/tilt.schema.json).
    """
    return {_topic(entity_id, "driver"): json.dumps(driver_key)}


def _topic(entity_id: str, prop: str) -> str:
    return f"renewvan/tilt/{entity_id}/{prop}"


def _state_path(entity_id: str, state_dir: Path = _STATE_DIR) -> Path:
    return state_dir / f"{entity_id}.json"


def load_offsets(entity_id: str, state_dir: Path = _STATE_DIR) -> tuple[float, float]:
    """Load (roll_offset, pitch_offset) from state, or (0, 0) if absent/unreadable."""
    path = _state_path(entity_id, state_dir)
    if not path.exists():
        return 0.0, 0.0
    try:
        data = json.loads(path.read_text())
        return float(data.get("roll_offset_deg", 0.0)), float(data.get("pitch_offset_deg", 0.0))
    except (OSError, ValueError, TypeError) as e:
        logger.warning("Failed to load state for %s: %s", entity_id, e)
        return 0.0, 0.0


def save_offsets(entity_id: str, roll: float, pitch: float, state_dir: Path = _STATE_DIR) -> bool:
    """Persist offsets. Returns False (and logs) on failure -- the in-memory
    zero still applies until restart, so the caller must not treat this as fatal."""
    try:
        state_dir.mkdir(exist_ok=True, parents=True)
        _state_path(entity_id, state_dir).write_text(
            json.dumps({"roll_offset_deg": roll, "pitch_offset_deg": pitch})
        )
        return True
    except OSError as e:
        logger.error("Failed to save state for %s: %s", entity_id, e)
        return False


def compute_sample(
    accel: tuple[float, float, float], driver: DriverConfig, state: TiltState, axis_map: AxisMap
) -> None:
    """Fold one accelerometer sample into the smoothed attitude in `state`.

    Samples outside the gravity band are dynamic (braking, cornering, bumps)
    and are skipped once an estimate exists. The very first sample always
    seeds the estimate so the driver publishes something at startup.
    """
    van = remap_axes(accel, axis_map)
    roll, pitch = roll_pitch_deg(van)  # ValueError on a zero vector
    if state.roll_smoothed is not None and not plausible_gravity(
        van, driver.gravity_min_g, driver.gravity_max_g
    ):
        return
    state.roll_smoothed = ema(state.roll_smoothed, roll, driver.smoothing_alpha)
    state.pitch_smoothed = ema(state.pitch_smoothed, pitch, driver.smoothing_alpha)


def corrected(state: TiltState) -> tuple[float, float] | None:
    """Level-corrected (roll, pitch), or None before the first sample."""
    if state.roll_smoothed is None or state.pitch_smoothed is None:
        return None
    return (
        apply_offset(state.roll_smoothed, state.roll_offset),
        apply_offset(state.pitch_smoothed, state.pitch_offset),
    )


def should_publish_angles(
    roll: float, pitch: float, state: TiltState, driver: DriverConfig, now: float
) -> bool:
    if state.last_published_roll is None or state.last_published_pitch is None:
        return True
    if now - state.last_publish_at >= driver.heartbeat_s:
        return True
    return (
        abs(roll - state.last_published_roll) >= driver.publish_deadband_deg
        or abs(pitch - state.last_published_pitch) >= driver.publish_deadband_deg
    )


def _publish_status(
    publisher: Publisher, driver: DriverConfig, state: TiltState, status: str
) -> None:
    if status != state.last_status:
        publisher.publish(_topic(driver.id, "status"), json.dumps(status))
        state.last_status = status


def read_and_publish(
    publisher: Publisher,
    imu: MPU6050,
    driver: DriverConfig,
    state: TiltState,
    axis_map: AxisMap,
    now: float,
) -> None:
    try:
        accel = imu.read_accel_g()
        compute_sample(accel, driver, state, axis_map)
    except (OSError, ValueError) as e:
        state.consecutive_errors += 1
        logger.warning(
            "Read failed (%d/%d): %s", state.consecutive_errors, driver.error_threshold, e
        )
        if state.consecutive_errors >= driver.error_threshold:
            _publish_status(publisher, driver, state, "sensor_error")
        return

    state.consecutive_errors = 0
    _publish_status(publisher, driver, state, "ok")

    angles = corrected(state)
    if angles is None:
        return
    roll, pitch = angles
    if not should_publish_angles(roll, pitch, state, driver, now):
        return
    publisher.publish(_topic(driver.id, "roll_deg"), json.dumps(round(roll, 2)))
    publisher.publish(_topic(driver.id, "pitch_deg"), json.dumps(round(pitch, 2)))
    state.last_published_roll = roll
    state.last_published_pitch = pitch
    state.last_publish_at = now
    logger.debug("roll=%.2f pitch=%.2f", roll, pitch)


def handle_level_zero_command(
    publisher: Publisher,
    driver: DriverConfig,
    state: TiltState,
    payload: str,
    state_dir: Path = _STATE_DIR,
) -> None:
    """Handle renewvan/tilt/<id>/level_zero/set.

    Any payload captures the current smoothed attitude as the new level;
    "reset"/"null" clears it. The republish is immediate so subscribers see
    0/0 straight away rather than at the next deadband crossing.
    """
    if payload.strip().strip('"').lower() in ("reset", "null"):
        state.roll_offset, state.pitch_offset = 0.0, 0.0
        logger.info("Level zero reset")
    else:
        if state.roll_smoothed is None or state.pitch_smoothed is None:
            logger.error("level_zero/set ignored: no sensor reading yet")
            return
        state.roll_offset = state.roll_smoothed
        state.pitch_offset = state.pitch_smoothed
        logger.info(
            "Level zero set: roll_offset=%.2f pitch_offset=%.2f",
            state.roll_offset,
            state.pitch_offset,
        )
    save_offsets(driver.id, state.roll_offset, state.pitch_offset, state_dir)
    state.last_published_roll = None  # force the next read to publish


def run(config: AppConfig) -> None:
    driver = config.driver
    axis_map = AxisMap(driver.axis_x, driver.axis_y, driver.axis_z)
    state = TiltState()
    state.roll_offset, state.pitch_offset = load_offsets(driver.id, driver.state_dir)

    info_json = build_info()
    publisher = Publisher(
        config.mqtt, info_json, identity_topics(driver.id, json.loads(info_json)["driver"])
    )
    publisher.connect()
    imu = MPU6050(bus_number=driver.i2c_bus, address=driver.i2c_address)
    publisher.subscribe(
        _topic(driver.id, "level_zero/set"),
        lambda payload: handle_level_zero_command(
            publisher, driver, state, payload, driver.state_dir
        ),
    )

    try:
        while True:
            try:
                read_and_publish(publisher, imu, driver, state, axis_map, time.time())
            except Exception:
                logger.exception("Unexpected failure in read/publish")
            time.sleep(driver.update_interval_ms / 1000)
    except KeyboardInterrupt:
        logger.info("Shutting down")
    finally:
        imu.close()
        publisher.disconnect()
