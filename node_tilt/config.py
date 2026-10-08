"""Layered INI config: config.default.ini (shipped) + config.ini (local
override).

Same two-file pattern as every renewvan/node-* repo (see
node-tank/node_tank/config.py): config.default.ini ships every key with a
safe default and is never edited at an install; config.ini is a gitignored,
install-local file that overrides only what differs.
"""

from __future__ import annotations

import configparser
import logging
from dataclasses import dataclass
from pathlib import Path

from node_tilt.tilt_math import AxisMap

logger = logging.getLogger(__name__)

DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.default.ini"
DEFAULT_STATE_DIR = Path(__file__).resolve().parent.parent / "state"


@dataclass(frozen=True)
class MqttConfig:
    host: str
    port: int
    username: str | None
    password: str | None
    client_id: str
    reconnect_min_delay: float
    reconnect_max_delay: float


@dataclass(frozen=True)
class NodeConfig:
    id: str  # topic path segment, e.g. renewvan/tilt/<id>/...
    update_interval_ms: int
    i2c_bus: int
    i2c_address: int
    # Sensor-axis -> van-axis mapping (see tilt_math.AxisMap).
    axis_x: str
    axis_y: str
    axis_z: str
    # EMA smoothing factor in (0, 1]; 1.0 disables smoothing.
    smoothing_alpha: float
    # Samples with |accel| outside [min, max] g are dynamic (braking,
    # cornering, bumps) and are excluded from the smoothed estimate.
    gravity_min_g: float
    gravity_max_g: float
    # Publish only when roll/pitch moved at least this many degrees since the
    # last publish (plus a heartbeat every heartbeat_s); 0 publishes every read.
    publish_deadband_deg: float
    heartbeat_s: float
    # Consecutive failed reads before status flips to sensor_error.
    error_threshold: int
    # Where the level-zero offsets persist (<state_dir>/<id>.json). Must be a
    # persistent volume in a container, or the install zero is lost whenever
    # the container is recreated.
    state_dir: Path


@dataclass(frozen=True)
class AppConfig:
    mqtt: MqttConfig
    node: NodeConfig


def load_config(
    default_path: Path = DEFAULT_CONFIG_PATH, local_path: Path | None = None
) -> AppConfig:
    """Load config.default.ini, then layer config.ini (if present) on top."""
    parser = configparser.ConfigParser()
    parser.read(default_path)
    if local_path is not None and local_path.exists():
        parser.read(local_path)
    else:
        logger.info("No local config override at %s; using defaults only", local_path)

    mqtt = parser["mqtt"]
    node = parser["node"]

    node_config = NodeConfig(
        id=node.get("id", "tilt"),
        update_interval_ms=node.getint("update_interval_ms", 500),
        i2c_bus=node.getint("i2c_bus", 1),
        i2c_address=int(node.get("i2c_address", "0x68"), 0),
        axis_x=node.get("axis_x", "+x").strip(),
        axis_y=node.get("axis_y", "+y").strip(),
        axis_z=node.get("axis_z", "+z").strip(),
        smoothing_alpha=node.getfloat("smoothing_alpha", 0.2),
        gravity_min_g=node.getfloat("gravity_min_g", 0.8),
        gravity_max_g=node.getfloat("gravity_max_g", 1.25),
        publish_deadband_deg=node.getfloat("publish_deadband_deg", 0.1),
        heartbeat_s=node.getfloat("heartbeat_s", 30.0),
        error_threshold=node.getint("error_threshold", 3),
        state_dir=Path(node.get("state_dir", str(DEFAULT_STATE_DIR))),
    )
    _validate(node_config)

    return AppConfig(
        mqtt=MqttConfig(
            host=mqtt.get("host", "localhost"),
            port=mqtt.getint("port", 1883),
            username=mqtt.get("username") or None,
            password=mqtt.get("password") or None,
            client_id=mqtt.get("client_id", "node-tilt"),
            reconnect_min_delay=mqtt.getfloat("reconnect_min_delay", 1.0),
            reconnect_max_delay=mqtt.getfloat("reconnect_max_delay", 30.0),
        ),
        node=node_config,
    )


def _validate(node: NodeConfig) -> None:
    """Fail at startup on config that would otherwise misbehave at runtime."""
    if not 0.0 < node.smoothing_alpha <= 1.0:
        raise ValueError(f"smoothing_alpha must be in (0, 1]; got {node.smoothing_alpha}")
    if not 0.0 < node.gravity_min_g < node.gravity_max_g:
        raise ValueError(
            f"need 0 < gravity_min_g < gravity_max_g; got "
            f"{node.gravity_min_g}..{node.gravity_max_g}"
        )
    if node.update_interval_ms <= 0:
        raise ValueError(f"update_interval_ms must be > 0; got {node.update_interval_ms}")
    if node.publish_deadband_deg < 0 or node.heartbeat_s <= 0:
        raise ValueError("publish_deadband_deg must be >= 0 and heartbeat_s > 0")
    if node.error_threshold < 1:
        raise ValueError(f"error_threshold must be >= 1; got {node.error_threshold}")
    # Raises ValueError with a precise message on a bad/degenerate mapping.
    AxisMap(node.axis_x, node.axis_y, node.axis_z)
