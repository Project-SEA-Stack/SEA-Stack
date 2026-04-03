#!/usr/bin/env bash
# Packaged / Unix test driver (counterpart to RUN-TESTS.ps1).
# Resolves install root from this script location: <install>/tests/RUN-TESTS.sh -> <install>
set -euo pipefail

PYTHON="python3"
NO_VENV=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --python) PYTHON="$2"; shift 2 ;;
    --no-venv) NO_VENV=1; shift ;;
    *) break ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TESTS="${INSTALL_ROOT}/tests"
DEMOS="${INSTALL_ROOT}/demos"
BIN="${INSTALL_ROOT}/bin"

if ! "${PYTHON}" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)' 2>/dev/null; then
  echo "Python 3.10 or newer is required to run the packaged tests." >&2
  echo "Interpreter: ${PYTHON}" >&2
  "${PYTHON}" --version >&2 2>/dev/null || true
  exit 1
fi

if [[ ! -d "${TESTS}" ]]; then
  echo "Tests not found at ${TESTS}" >&2
  exit 1
fi
if [[ ! -d "${DEMOS}" ]]; then
  echo "Demos not found at ${DEMOS}" >&2
  exit 1
fi

EXE=""
if [[ -x "${BIN}/run_seastack" ]]; then
  EXE="${BIN}/run_seastack"
elif [[ -f "${BIN}/run_seastack.exe" ]]; then
  EXE="${BIN}/run_seastack.exe"
else
  echo "Executable not found in ${BIN} (expected run_seastack or run_seastack.exe)" >&2
  exit 2
fi

export PATH="${BIN}:${PATH}"
export SEASTACK_DATA_DIR="${INSTALL_ROOT}/data"
export SS_RUN_EXE="${EXE}"
export SS_DEMOS_DIR="${DEMOS}"

cd "${TESTS}"

RUNNER="${PYTHON}"
if [[ "${NO_VENV}" -eq 0 ]]; then
  if [[ -x "${TESTS}/.venv/bin/python" ]]; then
    RUNNER="${TESTS}/.venv/bin/python"
  elif [[ -t 0 ]]; then
    reqs=()
    if [[ -f "${TESTS}/requirements.txt" ]]; then
      while IFS= read -r line || [[ -n "${line}" ]]; do
        [[ -z "${line}" || "${line}" =~ ^[[:space:]]*# ]] && continue
        reqs+=("${line}")
      done < "${TESTS}/requirements.txt"
    fi
    echo ""
    echo "SEA-Stack Test Environment"
    echo "  Tests folder : ${TESTS}"
    echo "  Python       : ${PYTHON}"
    if [[ "${#reqs[@]}" -gt 0 ]]; then
      echo "  Will install : ${reqs[*]}"
      echo "  Source       : PyPI via pip"
    fi
    read -r -p "Create local venv at '.venv' and install requirements? [y/N] " ans
    if [[ "${ans}" =~ ^[Yy] ]]; then
      if [[ ! -d "${TESTS}/.venv" ]]; then
        "${PYTHON}" -m venv "${TESTS}/.venv"
      fi
      "${TESTS}/.venv/bin/python" -m pip install --upgrade pip
      if [[ -f "${TESTS}/requirements.txt" ]]; then
        "${TESTS}/.venv/bin/python" -m pip install -r "${TESTS}/requirements.txt"
      fi
      RUNNER="${TESTS}/.venv/bin/python"
    fi
  fi
fi

exec "${RUNNER}" "${TESTS}/run_tests.py" --all --exe "${EXE}" --demos-dir "${DEMOS}"
