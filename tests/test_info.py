import json
from pathlib import Path

import pytest

from node_tilt.info import build_info

MANIFEST = {
    "node": "tilt",
    "product": "Tilt sensor",
    "source": "MPU-6050 IMU over I2C",
    "publishes": ["tilt"],
}

REAL_SERIAL = "10000000abcd1234"


@pytest.fixture
def manifest_path(tmp_path: Path) -> Path:
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(MANIFEST))
    return path


def cpuinfo(tmp_path: Path, text: str) -> Path:
    path = tmp_path / "cpuinfo"
    path.write_text(text)
    return path


def pi_cpuinfo(serial_line: str) -> str:
    return f"processor\t: 0\nModel name\t: ARMv8\n{serial_line}\nModel\t\t: Raspberry Pi 4\n"


def info(manifest_path: Path, cpuinfo_path: Path, env: dict[str, str]) -> dict:
    return json.loads(build_info(manifest_path=manifest_path, cpuinfo_path=cpuinfo_path, env=env))


def test_payload_carries_manifest_fields_and_stamped_version(manifest_path, tmp_path) -> None:
    payload = info(
        manifest_path,
        cpuinfo(tmp_path, pi_cpuinfo(f"Serial\t\t: {REAL_SERIAL}")),
        {"NODE_VERSION": "v0.4.2"},
    )
    assert payload == {**MANIFEST, "version": "v0.4.2", "host_serial": REAL_SERIAL}


@pytest.mark.parametrize("env", [{}, {"NODE_VERSION": ""}])
def test_version_is_dev_when_unset_or_empty(manifest_path, tmp_path, env) -> None:
    payload = info(manifest_path, cpuinfo(tmp_path, pi_cpuinfo("")), env)
    assert payload["version"] == "dev"


@pytest.mark.parametrize(
    "text",
    [
        pi_cpuinfo(""),  # no Serial line
        pi_cpuinfo("Serial\t\t: "),  # empty
        pi_cpuinfo("Serial\t\t: 0000000000000000"),  # all zeros
        pi_cpuinfo("Serial\t\t:    "),  # whitespace only
    ],
    ids=["missing", "empty", "all-zeros", "blank"],
)
def test_host_serial_is_omitted_when_absent_empty_or_all_zeros(
    manifest_path, tmp_path, text
) -> None:
    payload = info(manifest_path, cpuinfo(tmp_path, text), {})
    assert "host_serial" not in payload


def test_host_serial_is_omitted_when_cpuinfo_is_unreadable(manifest_path, tmp_path) -> None:
    payload = info(manifest_path, tmp_path / "does-not-exist", {})
    assert "host_serial" not in payload
    assert payload["node"] == "tilt"


def test_host_serial_survives_undecodable_bytes_elsewhere_in_cpuinfo(
    manifest_path, tmp_path
) -> None:
    path = tmp_path / "cpuinfo"
    path.write_bytes(b"Model name\t: \xff\xfe\nSerial\t\t: " + REAL_SERIAL.encode() + b"\n")
    assert info(manifest_path, path, {})["host_serial"] == REAL_SERIAL


def test_version_and_serial_in_manifest_file_are_not_trusted(tmp_path) -> None:
    # The schema forbids these in the file; the stamp is the only source.
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps({**MANIFEST, "version": "v9.9.9", "host_serial": "fake"}))
    payload = info(path, tmp_path / "missing", {"NODE_VERSION": "v1.0.0"})
    assert payload["version"] == "v1.0.0"
    assert "host_serial" not in payload
