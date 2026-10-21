#!/usr/bin/env bash
# Validates manifest.json against the hub's device-manifest schema.
#
# Usage: scripts/validate-manifest.sh [manifest.json]
#
# The schema is fetched from renewvan/hub at a PINNED ref, so a hub change
# can never break this repo's CI until the pin is bumped on purpose. Set
# MANIFEST_SCHEMA to a local file to validate against that instead (used by
# the gate's own test; no network).
set -euo pipefail
cd "$(dirname "$0")/.."

# Bump deliberately, together with any manifest change the new schema needs.
HUB_REF="a334f02d3c82a11c5cf93b345acfde740b1e0d53"
SCHEMA_URL="https://raw.githubusercontent.com/renewvan/hub/${HUB_REF}/schema/device-manifest.schema.json"

manifest="${1:-manifest.json}"
schema="${MANIFEST_SCHEMA:-}"

if [ -z "$schema" ]; then
  # ajv-cli picks its parser from the file extension: an extensionless temp file fails
  # with "Unexpected token ':'", so the schema needs a .json name inside a temp dir.
  tmpdir="$(mktemp -d)"
  trap 'rm -rf "$tmpdir"' EXIT
  schema="${tmpdir}/device-manifest.schema.json"
  if ! curl --fail --silent --location "$SCHEMA_URL" --output "$schema"; then
    echo "error: cannot fetch the device-manifest schema at hub ref ${HUB_REF}" >&2
    echo "       ${SCHEMA_URL}" >&2
    echo "       (does that ref contain schema/device-manifest.schema.json?)" >&2
    exit 2
  fi
fi

npx --yes ajv-cli@5 validate -s "$schema" -d "$manifest"
