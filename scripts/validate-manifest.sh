#!/usr/bin/env bash
# Validates manifest.json against the hub's node-manifest schema.
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
# NOTE: this is hub `main` as of writing. renewvan/hub#36 (the node-manifest
# schema) had not landed yet, so the file does not exist at this ref: bump it
# to the commit that adds schema/node-manifest.schema.json.
HUB_REF="e5808af4eb85ef9d1166bf7455c7432305c5e86b"
SCHEMA_URL="https://raw.githubusercontent.com/renewvan/hub/${HUB_REF}/schema/node-manifest.schema.json"

manifest="${1:-manifest.json}"
schema="${MANIFEST_SCHEMA:-}"

if [ -z "$schema" ]; then
  schema="$(mktemp)"
  trap 'rm -f "$schema"' EXIT
  if ! curl --fail --silent --location "$SCHEMA_URL" --output "$schema"; then
    echo "error: cannot fetch the node-manifest schema at hub ref ${HUB_REF}" >&2
    echo "       ${SCHEMA_URL}" >&2
    echo "       (does that ref contain schema/node-manifest.schema.json? see hub#36)" >&2
    exit 2
  fi
fi

npx --yes ajv-cli@5 validate -s "$schema" -d "$manifest"
