# driver-tilt

Native MQTT Driver for van inclination: reads a GY-521 (MPU-6050) accelerometer
over I2C and publishes `roll_deg` / `pitch_deg` to the renewvan bus. Owns the
sensing directly (no feed remap), same shape as `renewvan/driver-tank`: pure
math (`tilt_math.py`), a layered `config.default.ini`/`config.ini` loader
(`config.py`), a persistent MQTT publisher with LWT (`publisher.py`), a thin
publish loop (`driver.py`), a systemd unit, and its own Dockerfile + CI.

Roll/pitch come from the accelerometer alone. A parked or driving van is
quasi-static for a level indicator, and a gyro would only add bias drift.
Samples taken under braking/cornering/bumps (acceleration magnitude outside
`gravity_min_g`..`gravity_max_g`) are excluded from the smoothed estimate.

## Topics published

Retained, under `renewvan/tilt/<id>/` (default id `tilt`). One identity field,
the rest live:

- `driver` — JSON string, always `"tilt"`: this Driver's key, the same slug as
  the `renewvan/tilt/info` segment, so consumers can attribute the instance to
  its owning Driver. Published on **every connect, after `info` and before
  `online`**, so a retained copy exists the moment the Driver is seen online.
  Consumers must tolerate it being absent (an older Driver).
- `roll_deg` — number, degrees, van frame, positive = left side down. 0 = level.
- `pitch_deg` — number, degrees, van frame, positive = nose up. 0 = level.
- `status` — `ok` / `sensor_error` (after `error_threshold` consecutive failed
  I2C reads or all-zero accelerometer reads; angles keep their last
  published value while in error). Published on transition only. A supply dip
  puts the MPU-6050 back in its power-on sleep state, where it reads all zeros
  or keeps returning its last sample frozen. The driver checks the chip's SLEEP
  bit before each read, and treats an all-zero sample the same way: it wakes
  the chip again (logged at WARNING) before counting a failure. Repeated
  warnings mean an unstable 5 V supply.

`roll_deg`/`pitch_deg` are republished when either moves at least
`publish_deadband_deg` since the last publish, and on a `heartbeat_s` heartbeat
so the retained value never goes silently stale.

**Command topic** (unretained):

- `renewvan/tilt/<id>/level_zero/set` — any payload captures the current
  smoothed attitude as the new level (park the van level first). Payload
  `reset` clears it. The offsets persist in `<state_dir>/<id>.json` (default
  `state/` next to the package), so a restart keeps the install zero, and the
  next read republishes immediately. **In a container, mount a persistent
  volume and set `state_dir` to it** (hub's compose mounts one at `/app/state`),
  or the zero is lost whenever the container is recreated.

**Driver liveness:** `renewvan/tilt/health` — `online`/`offline` via MQTT LWT
(3 segments, not 4, same as driver-tank).

**Driver info:** `renewvan/tilt/info` — retained JSON describing this Driver:
`driver`, `product`, `source`, `publishes` (and `manufacturer`/`homepage`) from
`manifest.json`, plus `version` and, when available, `board_serial`. It is
published on **every connect, immediately before `online`**, and stays
retained when the Driver goes offline (it is never cleared, not even by the
last will), so consumers can show "offline, last seen vX".

- **`manifest.json`** (repo root) is the one place the static fields live. It
  must not contain `version` or `board_serial`. The image build copies it into
  the package (`driver_tilt/manifest.json`); the Driver reads that copy, and falls
  back to the repo-root file when run from a checkout. CI validates it
  against the hub's `driver-manifest` schema through hub's reusable manifest
  workflow (`.github/workflows/manifest.yml`; its `hub-ref` is the same pinned hub
  commit as the workflow itself, bump both together).
- **`version`** is the git release tag, and nothing else: `release.yml` passes
  it as the `DRIVER_VERSION` build arg, which the image exposes as an
  environment variable the Driver reads. When it is unset or empty (a checkout
  run, a local `docker build`) the Driver reports the literal `dev`. There is
  no `version` file or `__version__`; `pyproject.toml`'s version is a fixed
  placeholder that nothing reads.
- **`board_serial`** is the `Serial` line of the board's `/proc/cpuinfo`
  (readable from a default container, so no compose or Dockerfile change),
  read once at start. It is omitted, never faked, when the file is
  unreadable or the serial is missing, empty or all zeros.

`driver_tilt/info.py` is deliberately copied into each renewvan Python Driver
rather than shared as a package (see hub ADR 0001).

Whether the van counts as "level" is a tolerance the dashboard applies; the
Driver publishes angles only. Payload shapes match `hub`'s
`schema/tilt.schema.json`.

## Wiring

GY-521 on the Pi's I2C bus 1 — the same pins as the ADS1115 used by
driver-tank (they share the bus; the addresses differ, no extra wiring):

- VCC → 3.3V (Pi pin 1 or 17)
- GND → GND
- SDA → GPIO2/SDA (Pi pin 3)
- SCL → GPIO3/SCL (Pi pin 5)
- AD0 → GND/unconnected = address `0x68` (default); AD0 → 3.3V = `0x69`

Verify: `i2cdetect -y 1` lists `0x68` (and `0x48` for the ADS1115).

### Mounting orientation

Mount the board anywhere, in any orientation; tell the Driver which sensor axis
supplies each van axis (van frame: x forward, y left, z up) in `config.ini`:

```ini
[driver]
; each value: one of +x -x +y -y +z -z, every sensor axis used exactly once
axis_x = +x
axis_y = -y
axis_z = -z       ; board face-down, X toward the front
```

To find the mapping, read the raw accelerometer with the van level (e.g. with
`i2cget`/a short smbus2 script):

1. The sensor axis reading about ±1 g is vertical. Map it to `axis_z`: if it
   reads **+1 g** use the positive axis (`+z`), if **−1 g** use the negative
   one (`-z`).
2. Choose `axis_x` (toward the front of the van) and `axis_y` (toward the
   left) from the remaining two sensor axes, with signs such that the mapping
   is a rotation of the board, not a mirror image (right-handed: x × y = z).

A mirrored mapping is not caught by config validation. It shows up as roll or
pitch with the wrong sign when you tilt the van, so tilt it both ways (or
lift a wheel on a ramp) after the first install and confirm the signs.

After mounting, park the van level and publish to `level_zero/set` to remove
the residual mounting error.

## Development

```bash
git clone git@github.com:renewvan/driver-tilt.git
cd driver-tilt
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt -r requirements-test.txt
```

Conversion and decision logic are pure functions tested against fixture
values — no MQTT broker or MPU-6050 required. `python -m driver_tilt.main`
itself needs the real chip on I2C, so verify hardware-dependent changes
on-device per **Running**.

Before opening a PR: `pytest tests/ -v` must pass (same command CI runs), and
any change to a published field needs a matching `hub/schema/tilt.schema.json`
update in a `hub` PR (per
`hub/docs/adr/0001-compose-services-via-pinned-images-not-git-submodules.md`,
the repos are only coupled at the schema-version level).

## Configuration

`config.default.ini` ships every key with a default and documents each.
Copy `config.ini.example` to `config.ini` (gitignored) and override only what
differs — broker host and the axis mapping for your mounting orientation.
The settings live in the `[driver]` section (it was `[device]` before the
Device → Driver rename): an existing `config.ini` keeps working only once its
`[device]` header is renamed to `[driver]`; there is no fallback.
Invalid values (smoothing outside `(0, 1]`, a malformed or repeated axis,
`gravity_min_g >= gravity_max_g`, …) fail at startup.

## Running

```bash
pip install -r requirements.txt
python -m driver_tilt.main --config config.ini
```

Install `driver_tilt/systemd/driver-tilt.service` to run at boot
(`Restart=on-failure`) on the Pi the MPU-6050 is wired to. The Docker image
needs `/dev/i2c-1` passed through (`devices: ["/dev/i2c-1:/dev/i2c-1"]`), as
driver-tank's compose block does.

## Testing

```bash
pip install -r requirements.txt -r requirements-test.txt
pytest tests/ -v
```

## Releasing

Start a release from **Actions → Release → Run workflow**, or
`gh workflow run release.yml -f bump=minor` (`patch`, `major`, or `-f version=0.2.0`
to set it explicitly). One run works out the next version from the latest `v*` tag,
tags `main`, creates the GitHub release and builds and publishes
`ghcr.io/<owner>/driver-tilt:<tag>` (and `:latest`), which `hub`'s deployment compose file pins
by tag (never builds from source — see
`hub/docs/adr/0001-compose-services-via-pinned-images-not-git-submodules.md`).

A beta from a feature branch: dispatch the workflow on that branch with **prerelease**
ticked. It runs the tests, publishes only its own tag (never `:latest`) and writes nothing
to `main`. The process is shared by every renewvan repo and lives in `hub` (`hub/docs/adr/0015-one-release-process-via-reusable-workflows.md`);
this repo's `release.yml` only says what it ships.
