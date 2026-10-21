from pathlib import Path

import pytest

from driver_tilt.config import DEFAULT_STATE_DIR, load_config


def test_load_config_defaults() -> None:
    config = load_config()
    assert config.mqtt.port == 1883
    assert config.device.id == "tilt"
    assert config.device.i2c_address == 0x68
    assert (config.device.axis_x, config.device.axis_y, config.device.axis_z) == ("+x", "+y", "+z")


def test_load_config_local_override(tmp_path: Path) -> None:
    local = tmp_path / "config.ini"
    local.write_text("[mqtt]\nhost = 192.168.1.50\n")
    config = load_config(local_path=local)
    assert config.mqtt.host == "192.168.1.50"


def test_device_override_layers_over_defaults(tmp_path: Path) -> None:
    local = tmp_path / "config.ini"
    local.write_text("[device]\ni2c_address = 0x69\naxis_y = -y\naxis_z = -z\n")
    device = load_config(local_path=local).device
    assert device.i2c_address == 0x69
    assert (device.axis_x, device.axis_y, device.axis_z) == ("+x", "-y", "-z")
    assert device.update_interval_ms == 500  # untouched default


@pytest.mark.parametrize(
    "override",
    [
        "smoothing_alpha = 0",
        "smoothing_alpha = 1.5",
        "gravity_min_g = 1.3\ngravity_max_g = 1.2",
        "update_interval_ms = 0",
        "error_threshold = 0",
        "heartbeat_s = 0",
        "axis_x = +y",  # duplicates axis_y's sensor axis
        "axis_z = up",
    ],
)
def test_invalid_device_config_fails_at_load(tmp_path: Path, override: str) -> None:
    local = tmp_path / "config.ini"
    local.write_text(f"[device]\n{override}\n")
    with pytest.raises(ValueError):
        load_config(local_path=local)


def test_state_dir_defaults_to_package_state_and_is_overridable(tmp_path: Path) -> None:
    assert load_config().device.state_dir == DEFAULT_STATE_DIR
    local = tmp_path / "config.ini"
    local.write_text("[device]\nstate_dir = /data/tilt\n")
    assert load_config(local_path=local).device.state_dir == Path("/data/tilt")
