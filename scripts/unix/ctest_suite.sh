#!/usr/bin/env bash
# Run SEA-Stack CTest suites on macOS/Linux (parity with scripts/windows/run_*_tests.ps1).
# Usage: ./scripts/unix/ctest_suite.sh <suite> [options]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

SUITE=""
BUILD_DIR="build"
BUILD_TYPE="Release"
JOBS=0
QUIET=0
VERBOSE=0
NO_PDF=0
FILTER=""
RECOMPARE=0
LONG=0
BASELINE=""
NO_PYTHON=0

usage() {
  cat <<EOF
Usage: $(basename "$0") <suite> [options]

Suites:
  unit            CTest label: unit
  chrono-free     CTest label: chrono-free
  regression      CTest label: regression + Python regression report
  verification    CTest label: verification (excl. verification_report_generation) + report
  comparison      CTest label: comparison (excl. comparison_report_generation) + report
  benchmark       CTest label: benchmark (excl. benchmark_report_generation) + report

Options:
  --build-dir DIR      Build directory relative to repo root (default: build)
  --build-type TYPE    Release|Debug|RelWithDebInfo|MinSizeRel (default: Release)
  -j N                 Parallel jobs (0 = auto: CPU count)
  --quiet              ctest -Q
  --verbose            ctest -V
  --no-pdf             Skip --pdf for report scripts
  --filter MODEL       regression only: -R ^test_regression_<MODEL>_
  --recompare          regression only: pass --recompare to generate_regression_report.py
  --long               regression only: SEASTACK_LONG_TESTS=1 for ctest
  --baseline TAG       benchmark only: --baseline for generate_benchmark_report.py
  --no-python          Skip Python-backed reference/analysis CTest labels and post-suite reports
  -h, --help           This help

Run from any directory; paths are resolved from the repository root.
Python-backed reference/compare steps (not the whole C++ regression suite) need numpy
and matplotlib for the CMake Python3 interpreter; see:
  tests/regression/run_seastack/requirements.txt
EOF
}

if [[ $# -lt 1 ]]; then
  usage >&2
  exit 1
fi

SUITE="$1"
shift

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-dir) BUILD_DIR="$2"; shift 2 ;;
    --build-type) BUILD_TYPE="$2"; shift 2 ;;
    -j) JOBS="$2"; shift 2 ;;
    --quiet) QUIET=1; shift ;;
    --verbose) VERBOSE=1; shift ;;
    --no-pdf) NO_PDF=1; shift ;;
    --filter) FILTER="$2"; shift 2 ;;
    --recompare) RECOMPARE=1; shift ;;
    --long) LONG=1; shift ;;
    --baseline) BASELINE="$2"; shift 2 ;;
    --no-python) NO_PYTHON=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
  esac
done

BUILD_PATH="${REPO_ROOT}/${BUILD_DIR}"
if [[ ! -d "${BUILD_PATH}" ]]; then
  echo "[WARN] Build directory not found: ${BUILD_PATH}" >&2
  echo "       Run scripts/unix/build.sh first." >&2
  exit 1
fi

if [[ "${JOBS}" -eq 0 ]]; then
  if command -v nproc >/dev/null 2>&1; then
    WORKERS="$(nproc)"
  elif command -v sysctl >/dev/null 2>&1; then
    WORKERS="$(sysctl -n hw.ncpu 2>/dev/null || echo 4)"
  else
    WORKERS=4
  fi
  echo "Using ${WORKERS} parallel workers (auto)"
else
  WORKERS="${JOBS}"
  echo "Using ${WORKERS} parallel workers"
fi

# Bash 3.2 (macOS default) with set -u: expanding "${arr[@]}" on an empty array
# triggers "unbound variable" and aborts before ctest runs. Branch on flags instead.
inv_ctest() {
  if [[ "${QUIET}" -eq 1 ]]; then
    (cd "${REPO_ROOT}" && ctest -C "${BUILD_TYPE}" --test-dir "${BUILD_PATH}" \
      --output-on-failure -Q "$@")
  elif [[ "${VERBOSE}" -eq 1 ]]; then
    (cd "${REPO_ROOT}" && ctest -C "${BUILD_TYPE}" --test-dir "${BUILD_PATH}" \
      --output-on-failure -V "$@")
  else
    (cd "${REPO_ROOT}" && ctest -C "${BUILD_TYPE}" --test-dir "${BUILD_PATH}" \
      --output-on-failure "$@")
  fi
}

pandoc_wants_pdf() {
  [[ "${NO_PDF}" -eq 1 ]] && return 1
  command -v pandoc >/dev/null 2>&1
}

# Interpreter used by CTest (CMake Python3_EXECUTABLE); used for dep probe and reports.
PYTHON_FOR_TESTS="python3"
HAVE_PY_DEPS=1

# Sets PYTHON_FOR_TESTS from CMakeCache, HAVE_PY_DEPS from numpy+matplotlib import.
# Prints guidance when deps are missing, or when --no-python is set.
# Safe if python3 is missing or CMake's Python path is invalid: HAVE_PY_DEPS=0.
seastack_probe_python_deps() {
  local cache="${BUILD_PATH}/CMakeCache.txt"
  if [[ -f "${cache}" ]]; then
    local line cmake_py
    line="$(grep '^Python3_EXECUTABLE:' "${cache}" 2>/dev/null | head -1 || true)"
    if [[ -n "${line}" ]]; then
      cmake_py="${line#*=}"
      if [[ -n "${cmake_py}" && -x "${cmake_py}" ]]; then
        PYTHON_FOR_TESTS="${cmake_py}"
      fi
    fi
  fi
  if "${PYTHON_FOR_TESTS}" -c "import numpy, matplotlib" >/dev/null 2>&1; then
    HAVE_PY_DEPS=1
  else
    HAVE_PY_DEPS=0
  fi
  if [[ "${NO_PYTHON}" -eq 1 ]]; then
    echo "Note: --no-python - omitting Python-backed reference/analysis CTest labels and report scripts." >&2
  elif [[ "${HAVE_PY_DEPS}" -eq 0 ]]; then
    echo "----------------------------------------------------------------------" >&2
    echo "SEA-Stack: Python-backed reference/analysis steps need numpy and matplotlib." >&2
    echo "Interpreter: ${PYTHON_FOR_TESTS} (from CMake Python3_EXECUTABLE or python3)." >&2
    echo "C++ regression execute tests still run; reference/compare CTest steps will Skip until:" >&2
    echo "  ${PYTHON_FOR_TESTS} -m pip install -r tests/regression/run_seastack/requirements.txt" >&2
    echo "Or skip those steps:  ./scripts/unix/ctest_suite.sh ${SUITE} --no-python" >&2
    echo "----------------------------------------------------------------------" >&2
  fi
}

# Skip post-suite report if user asked --no-python or scientific stack is unavailable.
seastack_skip_python_report() {
  [[ "${NO_PYTHON}" -eq 1 || "${HAVE_PY_DEPS}" -eq 0 ]]
}

case "${SUITE}" in
  unit|chrono-free)
    echo ""
    echo ">> ${SUITE} tests (${WORKERS} workers)"
    inv_ctest -L "${SUITE}" -j "${WORKERS}"
    ;;

  regression)
    seastack_probe_python_deps
    echo ""
    if [[ -n "${FILTER}" ]]; then
      echo ">> regression tests (filter: ${FILTER}, ${WORKERS} workers)"
    else
      echo ">> regression tests (${WORKERS} workers)"
    fi
    prior_long="${SEASTACK_LONG_TESTS:-}"
    if [[ "${LONG}" -eq 1 ]]; then
      export SEASTACK_LONG_TESTS=1
      echo "   SEASTACK_LONG_TESTS=1 (long regression enabled)" >&2
    fi
    # Bash 3.2 + set -u: do not expand "${empty_array[@]}" (unbound variable). Branch instead.
    set +e
    if [[ -n "${FILTER}" ]]; then
      if [[ "${NO_PYTHON}" -eq 1 ]]; then
        inv_ctest -L regression -R "^test_regression_${FILTER}_" -LE reference -j "${WORKERS}"
      else
        inv_ctest -L regression -R "^test_regression_${FILTER}_" -j "${WORKERS}"
      fi
    else
      if [[ "${NO_PYTHON}" -eq 1 ]]; then
        inv_ctest -L regression -LE reference -j "${WORKERS}"
      else
        inv_ctest -L regression -j "${WORKERS}"
      fi
    fi
    ctest_exit=$?
    set -e
    if [[ "${LONG}" -eq 1 ]]; then
      if [[ -n "${prior_long}" ]]; then
        export SEASTACK_LONG_TESTS="${prior_long}"
      else
        unset SEASTACK_LONG_TESTS
      fi
    fi

    echo ""
    echo ">> Regression report"
    if seastack_skip_python_report; then
      echo "   Skipped (Python-backed report needs numpy/matplotlib, or use --no-python)." >&2
    else
      rep_py="${REPO_ROOT}/tests/regression/utilities/generate_regression_report.py"
      rep_args=("${PYTHON_FOR_TESTS}" "${rep_py}" --build-dir "${BUILD_DIR}" --config "${BUILD_TYPE}")
      [[ "${RECOMPARE}" -eq 1 ]] && rep_args+=(--recompare)
      [[ "${VERBOSE}" -eq 0 ]] && rep_args+=(--quiet)
      if pandoc_wants_pdf; then
        echo "   pandoc found -- requesting PDF" >&2
        rep_args+=(--pdf)
      elif [[ "${NO_PDF}" -eq 0 ]]; then
        echo "   pandoc not on PATH -- Markdown only" >&2
      fi
      (cd "${REPO_ROOT}" && "${rep_args[@]}")
    fi
    exit "${ctest_exit}"
    ;;

  verification)
    seastack_probe_python_deps
    echo ""
    echo ">> verification tests (${WORKERS} workers)"
    ver_py_args=(-E '^verification_report_generation$')
    [[ "${NO_PYTHON}" -eq 1 ]] && ver_py_args+=(-LE reference)
    set +e
    inv_ctest -L verification "${ver_py_args[@]}" -j "${WORKERS}"
    ctest_exit=$?
    set -e

    echo ""
    echo ">> Verification report"
    if seastack_skip_python_report; then
      echo "   Skipped (Python-backed report needs numpy/matplotlib, or use --no-python)." >&2
    else
      out="${BUILD_PATH}/bin/${BUILD_TYPE}/results/tests/verification/report"
      rep_py="${REPO_ROOT}/tests/verification/utilities/generate_verification_report.py"
      rep_args=("${PYTHON_FOR_TESTS}" "${rep_py}" --build-dir "${BUILD_DIR}" --output-dir "${out}")
      [[ "${VERBOSE}" -eq 0 ]] && rep_args+=(--quiet)
      if pandoc_wants_pdf; then
        echo "   pandoc found -- requesting PDF" >&2
        rep_args+=(--pdf)
      elif [[ "${NO_PDF}" -eq 0 ]]; then
        echo "   pandoc not on PATH -- Markdown only" >&2
      fi
      (cd "${REPO_ROOT}" && "${rep_args[@]}")
    fi
    exit "${ctest_exit}"
    ;;

  comparison)
    seastack_probe_python_deps
    echo ""
    echo ">> comparison tests (${WORKERS} workers)"
    cmp_py_args=(-E '^comparison_report_generation$')
    [[ "${NO_PYTHON}" -eq 1 ]] && cmp_py_args+=(-LE analysis)
    set +e
    inv_ctest -L comparison "${cmp_py_args[@]}" -j "${WORKERS}"
    ctest_exit=$?
    set -e

    echo ""
    echo ">> Comparison report"
    if seastack_skip_python_report; then
      echo "   Skipped (Python-backed report needs numpy/matplotlib, or use --no-python)." >&2
    else
      out="${BUILD_PATH}/bin/${BUILD_TYPE}/results/tests/comparison/report"
      rep_py="${REPO_ROOT}/tests/comparison/utilities/generate_comparison_report.py"
      rep_args=("${PYTHON_FOR_TESTS}" "${rep_py}" --build-dir "${BUILD_DIR}" --output-dir "${out}")
      [[ "${VERBOSE}" -eq 0 ]] && rep_args+=(--quiet)
      if pandoc_wants_pdf; then
        echo "   pandoc found -- requesting PDF" >&2
        rep_args+=(--pdf)
      elif [[ "${NO_PDF}" -eq 0 ]]; then
        echo "   pandoc not on PATH -- Markdown only" >&2
      fi
      (cd "${REPO_ROOT}" && "${rep_args[@]}")
    fi
    exit "${ctest_exit}"
    ;;

  benchmark)
    seastack_probe_python_deps
    echo ""
    echo ">> benchmark tests (sequential -j 1)"
    set +e
    inv_ctest -L benchmark -E '^benchmark_report_generation$' -j 1
    ctest_exit=$?
    set -e

    echo ""
    echo ">> Benchmark report"
    if seastack_skip_python_report; then
      echo "   Skipped (Python-backed report needs numpy/matplotlib, or use --no-python)." >&2
      rep_exit=0
    else
      out="${BUILD_PATH}/bin/${BUILD_TYPE}/results/tests/benchmark/report"
      hist="${REPO_ROOT}/data/benchmarks/history"
      rep_py="${REPO_ROOT}/tests/benchmark/utilities/generate_benchmark_report.py"
      rep_args=("${PYTHON_FOR_TESTS}" "${rep_py}" --build-dir "${BUILD_DIR}" --output-dir "${out}" --history-dir "${hist}")
      [[ -n "${BASELINE}" ]] && rep_args+=(--baseline "${BASELINE}")
      set +e
      (cd "${REPO_ROOT}" && "${rep_args[@]}")
      rep_exit=$?
      set -e
    fi
    if [[ "${ctest_exit}" -ne 0 ]]; then
      exit "${ctest_exit}"
    fi
    exit "${rep_exit}"
    ;;

  *)
    echo "Unknown suite: ${SUITE}" >&2
    usage >&2
    exit 1
    ;;
esac
