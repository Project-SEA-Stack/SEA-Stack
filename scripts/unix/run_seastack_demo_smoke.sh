#!/usr/bin/env bash
# Headless smoke test: RM3 irregular run_seastack demo completes without divergence.
# Usage:
#   ./scripts/unix/run_seastack_demo_smoke.sh
#   RUN_SEASTACK=/path/to/run_seastack ./scripts/unix/run_seastack_demo_smoke.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

BIN="${RUN_SEASTACK:-}"
if [[ -z "${BIN}" ]]; then
  for cand in \
    "${REPO_ROOT}/build/bin/Release/run_seastack" \
    "${REPO_ROOT}/build/bin/run_seastack" \
    "${REPO_ROOT}/build-verify2/bin/Release/run_seastack"; do
    if [[ -x "${cand}" ]]; then
      BIN="${cand}"
      break
    fi
  done
fi

if [[ -z "${BIN}" || ! -x "${BIN}" ]]; then
  echo "run_seastack not found. Build the project or set RUN_SEASTACK to the executable path." >&2
  exit 1
fi

DEMO="${REPO_ROOT}/data/demos/run_seastack/rm3/irregular_waves"
if [[ ! -d "${DEMO}" ]]; then
  echo "Demo directory missing: ${DEMO}" >&2
  exit 1
fi

BIN_DIR="$(cd "$(dirname "${BIN}")" && pwd)"
export DYLD_LIBRARY_PATH="${BIN_DIR}:${DYLD_LIBRARY_PATH:-}"

echo "Smoke: ${BIN} ${DEMO} --nogui --nobanner"
exec "${BIN}" "${DEMO}" --nogui --nobanner
