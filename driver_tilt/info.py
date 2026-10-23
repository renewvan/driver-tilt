"""Driver info: the retained payload on renewvan/tilt/info.

Copied (not shared) into every renewvan Python Driver, deliberately -- see
hub's ADR 0001. It loads the static manifest, then adds the two values that
only exist at runtime: the version stamped from the git release tag
(`DRIVER_VERSION`, `dev` when unset) and the board serial from /proc/cpuinfo.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Mapping

# Image build copies manifest.json beside this module; a checkout run falls
# back to the repo-root file.
_HERE = Path(__file__).parent
MANIFEST_PATH = _HERE / "manifest.json"
if not MANIFEST_PATH.is_file():
    MANIFEST_PATH = _HERE.parent / "manifest.json"
CPUINFO_PATH = Path("/proc/cpuinfo")


def board_serial(cpuinfo_path: Path = CPUINFO_PATH) -> str | None:
    """`Serial` line of /proc/cpuinfo, or None when unreadable, empty or all zeros."""
    try:
        lines = cpuinfo_path.read_text(errors="replace").splitlines()
    except OSError:
        return None
    for line in lines:
        key, _, value = line.partition(":")
        if key.strip() == "Serial":
            serial = value.strip()
            return serial if serial.strip("0") else None
    return None


def build_info(
    manifest_path: Path = MANIFEST_PATH,
    cpuinfo_path: Path = CPUINFO_PATH,
    env: Mapping[str, str] = os.environ,
) -> str:
    """Serialised Driver info: manifest + stamped version + board serial (if any)."""
    info = json.loads(manifest_path.read_text())
    info.pop("board_serial", None)  # runtime-only; never trust the file
    info["version"] = env.get("DRIVER_VERSION") or "dev"
    serial = board_serial(cpuinfo_path)
    if serial:
        info["board_serial"] = serial
    return json.dumps(info)
