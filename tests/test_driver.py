import json
import math
from dataclasses import replace
from pathlib import Path

import pytest

from node_tilt.config import load_config
from node_tilt.driver import (
    TiltState,
    compute_sample,
    corrected,
    handle_level_zero_command,
    load_offsets,
    read_and_publish,
    save_offsets,
    should_publish_angles,
)
from node_tilt.tilt_math import AxisMap

NODE = load_config().node
AXES = AxisMap()  # identity


def tilted(roll_deg: float = 0.0, pitch_deg: float = 0.0) -> tuple[float, float, float]:
    r, p = math.radians(roll_deg), math.radians(pitch_deg)
    # Inverse of roll_pitch_deg for a unit gravity vector.
    ax = -math.sin(p)
    ay = math.cos(p) * math.sin(r)
    az = math.cos(p) * math.cos(r)
    return ax, ay, az


class FakePublisher:
    def __init__(self) -> None:
        self.messages: list[tuple[str, str]] = []

    def publish(self, topic: str, payload: str) -> None:
        self.messages.append((topic, payload))

    def topics(self) -> list[str]:
        return [t for t, _ in self.messages]

    def last(self, topic: str) -> str:
        return [p for t, p in self.messages if t == topic][-1]


class FakeImu:
    def __init__(self, samples: list) -> None:
        self._samples = list(samples)

    def read_accel_g(self):
        item = self._samples.pop(0)
        if isinstance(item, Exception):
            raise item
        return item


def run_reads(pub, imu, node, state, n, start=0.0, step=0.5) -> None:
    for i in range(n):
        read_and_publish(pub, imu, node, state, AXES, start + i * step)


def test_first_read_publishes_attitude_and_ok_status() -> None:
    pub, state = FakePublisher(), TiltState()
    run_reads(pub, FakeImu([tilted(3.0, -2.0)]), NODE, state, 1)
    assert json.loads(pub.last("renewvan/tilt/tilt/roll_deg")) == pytest.approx(3.0, abs=0.01)
    assert json.loads(pub.last("renewvan/tilt/tilt/pitch_deg")) == pytest.approx(-2.0, abs=0.01)
    assert json.loads(pub.last("renewvan/tilt/tilt/status")) == "ok"


def test_unchanged_attitude_inside_deadband_is_not_republished_until_heartbeat() -> None:
    pub, state = FakePublisher(), TiltState()
    imu = FakeImu([tilted(2.0, 0.0)] * 100)
    run_reads(pub, imu, NODE, state, 10)  # 0..4.5 s, far below heartbeat_s=30
    assert pub.topics().count("renewvan/tilt/tilt/roll_deg") == 1
    read_and_publish(pub, imu, NODE, state, AXES, NODE.heartbeat_s + 1)
    assert pub.topics().count("renewvan/tilt/tilt/roll_deg") == 2


def test_status_only_republished_on_transition() -> None:
    pub, state = FakePublisher(), TiltState()
    run_reads(pub, FakeImu([tilted(1.0)] * 5), NODE, state, 5)
    assert pub.topics().count("renewvan/tilt/tilt/status") == 1


def test_motion_beyond_deadband_republishes() -> None:
    node = replace(NODE, smoothing_alpha=1.0)
    pub, state = FakePublisher(), TiltState()
    run_reads(pub, FakeImu([tilted(0.0), tilted(1.0)]), node, state, 2)
    assert pub.topics().count("renewvan/tilt/tilt/roll_deg") == 2


def test_dynamic_sample_is_ignored_once_estimate_exists() -> None:
    node = replace(NODE, smoothing_alpha=1.0)
    state = TiltState()
    compute_sample(tilted(2.0), node, state, AXES)
    # 1.6 g braking spike: outside the default 0.8..1.25 band.
    spike = (0.0, 1.2, 1.1)
    compute_sample(spike, node, state, AXES)
    assert state.roll_smoothed == pytest.approx(2.0, abs=0.01)


def test_first_sample_seeds_even_if_outside_gravity_band() -> None:
    state = TiltState()
    compute_sample((0.0, 0.0, 1.5), NODE, state, AXES)
    assert state.roll_smoothed is not None


def test_sensor_error_only_after_threshold_consecutive_failures() -> None:
    pub, state = FakePublisher(), TiltState()
    errs = [OSError("i2c")] * NODE.error_threshold
    run_reads(pub, FakeImu([tilted(0.0)] + errs), NODE, state, 1 + len(errs))
    statuses = [json.loads(p) for t, p in pub.messages if t.endswith("/status")]
    assert statuses == ["ok", "sensor_error"]


def test_failures_below_threshold_do_not_flip_status() -> None:
    pub, state = FakePublisher(), TiltState()
    seq = [tilted(0.0)] + [OSError("i2c")] * (NODE.error_threshold - 1) + [tilted(0.0)]
    run_reads(pub, FakeImu(seq), NODE, state, len(seq))
    statuses = [json.loads(p) for t, p in pub.messages if t.endswith("/status")]
    assert statuses == ["ok"]
    assert state.consecutive_errors == 0


def test_recovery_after_error_republishes_ok() -> None:
    pub, state = FakePublisher(), TiltState()
    seq = [tilted(0.0)] + [OSError("i2c")] * NODE.error_threshold + [tilted(0.0)]
    run_reads(pub, FakeImu(seq), NODE, state, len(seq))
    statuses = [json.loads(p) for t, p in pub.messages if t.endswith("/status")]
    assert statuses == ["ok", "sensor_error", "ok"]


def test_zero_vector_read_counts_as_failure() -> None:
    pub, state = FakePublisher(), TiltState()
    seq = [(0.0, 0.0, 0.0)] * NODE.error_threshold
    run_reads(pub, FakeImu(seq), NODE, state, len(seq))
    assert json.loads(pub.last("renewvan/tilt/tilt/status")) == "sensor_error"
    assert "renewvan/tilt/tilt/roll_deg" not in pub.topics()


def test_level_zero_command_zeroes_current_attitude_and_persists(tmp_path: Path) -> None:
    node = replace(NODE, smoothing_alpha=1.0)
    pub, state = FakePublisher(), TiltState()
    run_reads(pub, FakeImu([tilted(4.0, -1.5)]), node, state, 1)
    handle_level_zero_command(pub, node, state, "now", state_dir=tmp_path)
    assert corrected(state) == pytest.approx((0.0, 0.0), abs=0.01)
    assert load_offsets(node.id, tmp_path) == pytest.approx((4.0, -1.5), abs=0.01)
    # The next read must publish immediately (0/0), not wait for the heartbeat.
    before = pub.topics().count("renewvan/tilt/tilt/roll_deg")
    read_and_publish(pub, FakeImu([tilted(4.0, -1.5)]), node, state, AXES, 1.0)
    assert pub.topics().count("renewvan/tilt/tilt/roll_deg") == before + 1
    assert json.loads(pub.last("renewvan/tilt/tilt/roll_deg")) == pytest.approx(0.0, abs=0.01)


def test_level_zero_applies_to_later_motion(tmp_path: Path) -> None:
    node = replace(NODE, smoothing_alpha=1.0)
    pub, state = FakePublisher(), TiltState()
    run_reads(pub, FakeImu([tilted(4.0, 0.0)]), node, state, 1)
    handle_level_zero_command(pub, node, state, "", state_dir=tmp_path)
    compute_sample(tilted(6.0, 0.0), node, state, AXES)
    assert corrected(state)[0] == pytest.approx(2.0, abs=0.01)


def test_level_zero_reset_clears_offset(tmp_path: Path) -> None:
    node = replace(NODE, smoothing_alpha=1.0)
    pub, state = FakePublisher(), TiltState()
    run_reads(pub, FakeImu([tilted(4.0, 2.0)]), node, state, 1)
    handle_level_zero_command(pub, node, state, "go", state_dir=tmp_path)
    handle_level_zero_command(pub, node, state, '"reset"', state_dir=tmp_path)
    assert corrected(state) == pytest.approx((4.0, 2.0), abs=0.01)
    assert load_offsets(node.id, tmp_path) == (0.0, 0.0)


def test_level_zero_without_reading_is_ignored(tmp_path: Path) -> None:
    state = TiltState()
    handle_level_zero_command(FakePublisher(), NODE, state, "now", state_dir=tmp_path)
    assert (state.roll_offset, state.pitch_offset) == (0.0, 0.0)
    assert not (tmp_path / f"{NODE.id}.json").exists()


def test_offsets_roundtrip_and_corrupt_state_falls_back(tmp_path: Path) -> None:
    assert load_offsets("tilt", tmp_path) == (0.0, 0.0)
    assert save_offsets("tilt", 1.25, -0.5, tmp_path)
    assert load_offsets("tilt", tmp_path) == (1.25, -0.5)
    (tmp_path / "tilt.json").write_text("{not json")
    assert load_offsets("tilt", tmp_path) == (0.0, 0.0)


def test_should_publish_boundary_is_inclusive() -> None:
    state = TiltState(last_published_roll=0.0, last_published_pitch=0.0, last_publish_at=0.0)
    node = replace(NODE, publish_deadband_deg=0.5)
    assert not should_publish_angles(0.49, 0.0, state, node, 1.0)
    assert should_publish_angles(0.5, 0.0, state, node, 1.0)
    assert should_publish_angles(0.0, -0.5, state, node, 1.0)


def test_run_persists_and_restores_level_zero_via_the_configured_state_dir(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # run() once ignored node.state_dir and always used the package-relative
    # default, so in a container the install zero vanished on every recreate.
    # Drive run() itself: set a zero through the subscribed command, "restart",
    # and require the new run() to load it back from the configured dir.
    from node_tilt import driver

    config = load_config()
    node = replace(config.node, smoothing_alpha=1.0, state_dir=tmp_path / "mounted")
    config = replace(config, node=node)

    # BaseException: run() swallows `Exception` around each read (it must keep
    # publishing through a bad cycle), which would turn a stop signal into an
    # infinite loop.
    class StopLoop(BaseException):
        pass

    def one_run(samples, command: str | None) -> TiltState:
        captured: dict = {}

        class RunPublisher(FakePublisher):
            def __init__(self, _config, _info_json) -> None:
                super().__init__()

            def connect(self) -> None: ...
            def disconnect(self) -> None: ...

            def subscribe(self, topic: str, callback) -> None:
                captured["command"] = callback

        class RunImu(FakeImu):
            def __init__(self, **_kw) -> None:
                super().__init__(samples)

            def close(self) -> None: ...

        def fake_read_and_publish(pub, imu, node_, state, axis_map, now) -> None:
            captured["state"] = state
            read_and_publish(pub, imu, node_, state, axis_map, now)
            if command is not None:
                captured["command"](command)
            raise StopLoop

        monkeypatch.setattr(driver, "Publisher", RunPublisher)
        monkeypatch.setattr(driver, "MPU6050", RunImu)
        monkeypatch.setattr(driver, "read_and_publish", fake_read_and_publish)
        monkeypatch.setattr(driver.time, "sleep", lambda _s: None)
        with pytest.raises(StopLoop):
            driver.run(config)
        return captured["state"]

    default_file = driver.DEFAULT_STATE_DIR / f"{node.id}.json"
    assert not default_file.exists(), "test would be meaningless with a pre-existing zero"

    first = one_run([tilted(3.0, -1.0)], command="now")
    assert (first.roll_offset, first.pitch_offset) == pytest.approx((3.0, -1.0), abs=0.01)
    # The zero must land in the configured directory, never the package default.
    assert (node.state_dir / f"{node.id}.json").exists()
    assert not default_file.exists()

    second = one_run([tilted(3.0, -1.0)], command=None)
    assert (second.roll_offset, second.pitch_offset) == pytest.approx((3.0, -1.0), abs=0.01)
