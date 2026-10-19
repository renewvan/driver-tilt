"""The CI gate: scripts/validate-manifest.sh must reject a manifest carrying a
stamped field and accept the real one.

The real schema lives in renewvan/hub and CI fetches it at a pinned ref. These tests
run the actual script against a MINIMAL STAND-IN with the same contract (draft-07,
additionalProperties false, no `version`/`host_serial` property), so the gate's
pass/fail behaviour is proven without network access. It is deliberately not a copy
of the hub schema and is not kept in sync with it: the hub schema's exact rules are
hub's to test."""

import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
SCRIPT = ROOT / "scripts" / "validate-manifest.sh"
STAND_IN_SCHEMA = {
    "$schema": "http://json-schema.org/draft-07/schema#",
    "type": "object",
    "required": ["node", "product", "source", "publishes"],
    "additionalProperties": False,
    "properties": {
        "node": {"type": "string"},
        "product": {"type": "string"},
        "source": {"type": "string"},
        "publishes": {"type": "array", "minItems": 1, "items": {"type": "string"}},
        "manufacturer": {"type": "string"},
        "homepage": {"type": "string"},
    },
}

pytestmark = pytest.mark.skipif(
    shutil.which("npx") is None, reason="needs npx (ajv-cli), as the CI gate does"
)


def gate(manifest: Path, schema: Path) -> subprocess.CompletedProcess:
    return subprocess.run(
        [str(SCRIPT), str(manifest)],
        env={"PATH": os.environ["PATH"], "MANIFEST_SCHEMA": str(schema)},
        capture_output=True,
        text=True,
        timeout=120,
    )


@pytest.fixture
def schema(tmp_path) -> Path:
    path = tmp_path / "stand-in.schema.json"
    path.write_text(json.dumps(STAND_IN_SCHEMA))
    return path


def test_gate_passes_on_the_real_manifest(schema) -> None:
    result = gate(ROOT / "manifest.json", schema)
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("field", ["version", "host_serial"])
def test_gate_fails_on_a_manifest_containing_a_stamped_field(tmp_path, schema, field) -> None:
    manifest = json.loads((ROOT / "manifest.json").read_text())
    manifest[field] = "v1.0.0"
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(manifest))
    assert gate(path, schema).returncode != 0
