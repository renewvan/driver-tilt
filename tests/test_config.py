from pathlib import Path

import pytest

from driver_tilt.config import DEFAULT_STATE_DIR, load_config


def test_load_config_defaults() -> None:
    config = load_config()
    assert config.mqtt.port == 1883
    assert config.driver.id == "tilt"
    assert config.driver.i2c_address == 0x68
    assert (config.driver.axis_x, config.driver.axis_y, config.driver.axis_z) == ("+x", "+y", "+z")


def test_load_config_local_override(tmp_path: Path) -> None:
    local = tmp_path / "config.ini"
    local.write_text("[mqtt]\nhost = 192.168.1.50\n")
    config = load_config(local_path=local)
    assert config.mqtt.host == "192.168.1.50"


def test_driver_override_layers_over_defaults(tmp_path: Path) -> None:
    local = tmp_path / "config.ini"
    local.write_text("[driver]\ni2c_address = 0x69\naxis_y = -y\naxis_z = -z\n")
    driver_config = load_config(local_path=local).driver
    assert driver_config.i2c_address == 0x69
    assert (driver_config.axis_x, driver_config.axis_y, driver_config.axis_z) == ("+x", "-y", "-z")
    assert driver_config.update_interval_ms == 500  # untouched default


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
def test_invalid_driver_config_fails_at_load(tmp_path: Path, override: str) -> None:
    local = tmp_path / "config.ini"
    local.write_text(f"[driver]\n{override}\n")
    with pytest.raises(ValueError):
        load_config(local_path=local)


def test_state_dir_defaults_to_package_state_and_is_overridable(tmp_path: Path) -> None:
    assert load_config().driver.state_dir == DEFAULT_STATE_DIR
    local = tmp_path / "config.ini"
    local.write_text("[driver]\nstate_dir = /data/tilt\n")
    assert load_config(local_path=local).driver.state_dir == Path("/data/tilt")
