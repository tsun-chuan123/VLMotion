#!/usr/bin/env bash
set -euo pipefail

compose_file="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/docker-compose.yml"

if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
  cmd=(docker compose -f "$compose_file")
elif command -v docker-compose >/dev/null 2>&1; then
  cmd=(docker-compose -f "$compose_file")
else
  echo "Error: neither 'docker compose' nor 'docker-compose' found" >&2
  exit 1
fi

"${cmd[@]}" down

# Revoke local root X access if xhost exists (best-effort)
if command -v xhost >/dev/null 2>&1; then
  xhost -local:root >/dev/null 2>&1 || true
fi
