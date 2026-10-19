"""The CI gate: scripts/validate-manifest.sh must reject a manifest carrying a
stamped field and accept the real one. Runs the actual script with a local
schema (MANIFEST_SCHEMA), so no network is needed."""

import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
SCRIPT = ROOT / "scripts" / "validate-manifest.sh"
SCHEMA = Path(__file__).parent / "fixtures" / "node-manifest.schema.json"

pytestmark = pytest.mark.skipif(
    shutil.which("npx") is None, reason="needs npx (ajv-cli), as the CI gate does"
)


def gate(manifest: Path) -> subprocess.CompletedProcess:
    return subprocess.run(
        [str(SCRIPT), str(manifest)],
        env={"PATH": os.environ["PATH"], "MANIFEST_SCHEMA": str(SCHEMA)},
        capture_output=True,
        text=True,
        timeout=120,
    )


def test_gate_passes_on_the_real_manifest() -> None:
    result = gate(ROOT / "manifest.json")
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("field", ["version", "host_serial"])
def test_gate_fails_on_a_manifest_containing_a_stamped_field(tmp_path, field) -> None:
    manifest = json.loads((ROOT / "manifest.json").read_text())
    manifest[field] = "v1.0.0"
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(manifest))
    assert gate(path).returncode != 0
