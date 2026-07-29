#!/usr/bin/env bash
# SEA-Stack build driver for macOS/Linux. Windows: scripts/windows/build.ps1
# Same build-config.json schema and CMake flags as build.ps1 (keep in sync).
set -euo pipefail

SCRIPT_VERSION="1.0"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BUILD_DIR="${REPO_ROOT}/build"
DIAG_LOG="${BUILD_DIR}/build-diagnostics.log"

BUILD_TYPE="Release"
CLEAN=0
VERBOSE=0
CONFIGURE_ONLY=0
RECONFIGURE=0
NO_CONFIGURE=0
DO_TEST=0
TEST_LABEL=""
DO_PACKAGE=0
TARGET=""
GENERATOR_CLI=""
USE_CHRONO=1
NO_HYDRO_IO=0
MOORDYN=0
EXTERNAL=0
VSG=0
VEHICLE=0
SPH=0
DEMOS=0
NO_TESTS=0
NO_APPS=0
CONFIG_PATH="${REPO_ROOT}/build-config.json"
DOCTOR=0
HELP=0

log_diag() {
  mkdir -p "$(dirname "${DIAG_LOG}")"
  printf '%s\n' "$@" >>"${DIAG_LOG}"
}

run_captured() {
  local step="$1"
  shift
  log_diag "---- ${step} ----"
  log_diag "$*"
  set +e
  if [[ "${VERBOSE}" -eq 1 ]]; then
    (cd "${REPO_ROOT}" && "$@") 2>&1 | tee -a "${DIAG_LOG}"
    local code=${PIPESTATUS[0]}
  else
    (cd "${REPO_ROOT}" && "$@") >>"${DIAG_LOG}" 2>&1
    local code=$?
  fi
  set -e
  log_diag "Exit code: ${code}"
  return "${code}"
}

cached_generator() {
  local cache="${BUILD_DIR}/CMakeCache.txt"
  [[ -f "${cache}" ]] || return 1
  grep '^CMAKE_GENERATOR:INTERNAL=' "${cache}" | head -1 | cut -d= -f2- | tr -d '\r'
}

generator_is_multiconfig() {
  case "$1" in
    *"Visual Studio"*|*Xcode*) return 0 ;;
    *) return 1 ;;
  esac
}

expand_generator_alias() {
  local n="$1"
  local l
  l="$(printf '%s' "${n}" | tr '[:upper:]' '[:lower:]')"
  case "${l}" in
    ninja) echo "Ninja" ;;
    "") echo "" ;;
    *) echo "${n}" ;;
  esac
}

resolve_generator() {
  local cli_g config_g cached_g clean_req="$1"
  cli_g="$(expand_generator_alias "${GENERATOR_CLI}")"
  config_g="$(expand_generator_alias "${SEASTACK_CONFIG_GENERATOR:-}")"
  if [[ "${clean_req}" -eq 0 ]]; then
    cached_g="$(cached_generator || true)"
  else
    cached_g=""
  fi
  log_diag "Generator resolution: CLI='${cli_g}' Config='${config_g}' Cached='${cached_g}'"

  if [[ -n "${cli_g}" ]]; then
    if [[ -n "${cached_g}" && "${cached_g}" != "${cli_g}" ]]; then
      echo "[FAIL] Generator mismatch: build tree uses '${cached_g}' but --generator requests '${cli_g}'." >&2
      echo "       Run with --clean to reconfigure, or omit --generator." >&2
      return 1
    fi
    echo "${cli_g}"
    return 0
  fi
  if [[ "${clean_req}" -eq 0 && -n "${cached_g}" ]]; then
    echo "${cached_g}"
    return 0
  fi
  if [[ -n "${config_g}" ]]; then
    echo "${config_g}"
    return 0
  fi
  if command -v ninja >/dev/null 2>&1; then
    log_diag "Auto-detect: Ninja found on PATH"
    echo "Ninja"
    return 0
  fi
  echo "[FAIL] No CMake generator specified and 'ninja' not on PATH." >&2
  echo "       Install ninja, set \"Generator\" in build-config.json, or pass --generator." >&2
  return 1
}

usage() {
  cat <<EOF
SEA-Stack build (Unix) v${SCRIPT_VERSION}
  macOS/Linux counterpart to scripts/windows/build.ps1

Usage:
  ${REPO_ROOT}/scripts/unix/build.sh [options]

Options:
  --build-type TYPE     Release|Debug|RelWithDebInfo|MinSizeRel (default: Release)
  --clean               Remove build/ and build_*
  --verbose             Verbose CMake/build output
  --configure-only      Configure only
  --reconfigure         Force re-run CMake configure
  --no-configure        Build only (existing CMake cache required)
  --test                Run ctest after build
  --test-label LABEL    ctest -L (e.g. unit, benchmark)
  --package             cmake --install + cpack (ZIP)
  --target NAME         cmake --target
  --generator NAME      CMake -G (default: Ninja if on PATH, or JSON Generator)
  --no-chrono           Chrono-free build (no build-config.json required for Chrono)
  --no-hydro-io         Disable HDF5 hydro IO
  --moordyn             Enable MoorDyn
  --external            Enable out-of-process external force modules (Python PTO IPC)
                        (implied by --package so RM3/OSWEC external_pto demos work in the ZIP)
  --vsg                 Enable VSG (requires Chrono with VSG)
  --vehicle             Enable Chrono::Vehicle support (requires Chrono with Vehicle module)
  --sph                 Enable Chrono::FSI SPH simulations (requires Chrono with FSI_SPH module; needs CUDA)
  --demos               Build demo executables
  --no-tests            Skip tests target
  --no-apps             Skip application executables
  --config-path PATH    JSON config (default: build-config.json in repo root)
  --doctor              Print diagnostics and exit
  -h, --help            This help

Copy build-config.example.json to build-config.json and set ChronoDir.
See also: build-config.example.macos.json
EOF
}

# Keep in sync with the option cases in the parse loop below.
ALL_LONG_OPTS=(
  --build-type --clean --verbose --configure-only --reconfigure
  --no-configure --test --test-label --package --target --generator
  --no-chrono --no-hydro-io --moordyn --external --vsg --vehicle --sph --demos --no-tests
  --no-apps --config-path --doctor --help
)

# Levenshtein distance (for "did you mean" hints). Bash 3.2+ compatible.
levenshtein_distance() {
  local s="$1" t="$2"
  local m=${#s} n=${#t}
  local i j c del ins sub
  [[ "${m}" -eq 0 ]] && { echo "${n}"; return; }
  [[ "${n}" -eq 0 ]] && { echo "${m}"; return; }

  local -a prev curr
  for ((j = 0; j <= n; j++)); do
    prev[j]=$j
  done

  for ((i = 1; i <= m; i++)); do
    curr[0]=$i
    for ((j = 1; j <= n; j++)); do
      if [[ "${s:i - 1:1}" == "${t:j - 1:1}" ]]; then
        c=0
      else
        c=1
      fi
      del=$((prev[j] + 1))
      ins=$((curr[j - 1] + 1))
      sub=$((prev[j - 1] + c))
      curr[j]=${del}
      [[ ${ins} -lt ${curr[j]} ]] && curr[j]=${ins}
      [[ ${sub} -lt ${curr[j]} ]] && curr[j]=${sub}
    done
    for ((j = 0; j <= n; j++)); do
      prev[j]=${curr[j]}
    done
  done
  echo "${prev[n]}"
}

suggest_similar_long_opt() {
  local bad="$1"
  local opt d best=999
  local -a matches=()

  [[ "${bad}" != --* ]] && return 0

  for opt in "${ALL_LONG_OPTS[@]}"; do
    d="$(levenshtein_distance "${bad}" "${opt}")"
    if [[ "${d}" -lt "${best}" ]]; then
      best=${d}
      matches=("${opt}")
    elif [[ "${d}" -eq "${best}" ]]; then
      matches+=("${opt}")
    fi
  done

  # Allow one edit; allow two edits only for longer flags (reduces noise).
  if [[ "${best}" -eq 1 ]]; then
    printf '  Did you mean: %s ?\n' "${matches[*]}" >&2
  elif [[ "${best}" -eq 2 && ${#bad} -ge 8 ]]; then
    printf '  Did you mean: %s ?\n' "${matches[*]}" >&2
  fi
}

unknown_option() {
  local bad="$1"
  echo "" >&2
  echo "============================================================" >&2
  echo "ERROR: Unknown option: ${bad}" >&2
  echo "============================================================" >&2
  echo "" >&2
  suggest_similar_long_opt "${bad}"
  echo "" >&2
  echo "Run with --help for the full option list." >&2
  echo "============================================================" >&2
  echo "" >&2
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-type) BUILD_TYPE="$2"; shift 2 ;;
    --clean) CLEAN=1; shift ;;
    --verbose) VERBOSE=1; shift ;;
    --configure-only) CONFIGURE_ONLY=1; shift ;;
    --reconfigure) RECONFIGURE=1; shift ;;
    --no-configure) NO_CONFIGURE=1; shift ;;
    --test) DO_TEST=1; shift ;;
    --test-label) TEST_LABEL="$2"; shift 2 ;;
    --package) DO_PACKAGE=1; shift ;;
    --target) TARGET="$2"; shift 2 ;;
    --generator) GENERATOR_CLI="$2"; shift 2 ;;
    --no-chrono) USE_CHRONO=0; shift ;;
    --no-hydro-io) NO_HYDRO_IO=1; shift ;;
    --moordyn) MOORDYN=1; shift ;;
    --external) EXTERNAL=1; shift ;;
    --vsg) VSG=1; shift ;;
    --vehicle) VEHICLE=1; shift ;;
    --sph) SPH=1; shift ;;
    --demos) DEMOS=1; shift ;;
    --no-tests) NO_TESTS=1; shift ;;
    --no-apps) NO_APPS=1; shift ;;
    --config-path)
      if [[ "$2" = /* ]]; then CONFIG_PATH="$2"
      else CONFIG_PATH="${REPO_ROOT}/$2"
      fi
      shift 2
      ;;
    --doctor) DOCTOR=1; shift ;;
    -h|--help) HELP=1; shift ;;
    *)
      unknown_option "$1"
      ;;
  esac
done

if [[ "${HELP}" -eq 1 ]]; then
  usage
  exit 0
fi

if [[ "${NO_CONFIGURE}" -eq 1 && "${CONFIGURE_ONLY}" -eq 1 ]]; then
  echo "[FAIL] --no-configure and --configure-only conflict" >&2
  exit 1
fi
if [[ "${NO_CONFIGURE}" -eq 1 && "${RECONFIGURE}" -eq 1 ]]; then
  echo "[FAIL] --no-configure and --reconfigure conflict" >&2
  exit 1
fi

mkdir -p "${BUILD_DIR}"
: >"${DIAG_LOG}"
{
  echo "========================================"
  echo "SEA-Stack build diagnostics (Unix)"
  echo "Time: $(date '+%Y-%m-%d %H:%M:%S')"
  echo "Script: ${SCRIPT_VERSION}"
  echo "========================================"
} >>"${DIAG_LOG}"
log_diag "Repo root: ${REPO_ROOT}"

uc_flag="$([[ "${USE_CHRONO}" -eq 1 ]] && echo 1 || echo 0)"
set +e
_prep_cmd=(python3 "${REPO_ROOT}/scripts/seastack_build_prepare.py" "${REPO_ROOT}" "${CONFIG_PATH}" "${uc_flag}")
[[ "${DOCTOR}" -eq 1 ]] && _prep_cmd+=(doctor)
_py_out="$("${_prep_cmd[@]}")"
prep_code=$?
set -e
if [[ "${prep_code}" -ne 0 ]]; then
  exit "${prep_code}"
fi
eval "${_py_out}"

USE_VSG=0
if [[ "${USE_CHRONO}" -eq 1 ]]; then
  if [[ "${VSG}" -eq 1 && "${SEASTACK_HAS_VSG_IN_CHRONO}" -eq 1 ]]; then
    USE_VSG=1
  elif [[ "${VSG}" -eq 1 ]]; then
    echo "   [WARN] VSG requested; Chrono package may not list VSG — CMake will validate" >&2
    USE_VSG=1
  fi
else
  USE_VSG=0
fi

if [[ -z "${SEASTACK_HDF5_DIR:-}" && "${NO_HYDRO_IO}" -eq 0 ]]; then
  echo "   [WARN] HDF5 not found; HydroIO will be disabled." >&2
  if [[ "${USE_CHRONO}" -eq 1 ]]; then
    echo "          If Chrono was built without HDF5, set HDF5Dir in build-config.json." >&2
  else
    echo "          For Chrono-free builds, set HDF5Dir or use --no-hydro-io." >&2
  fi
  NO_HYDRO_IO=1
fi

if [[ "${MOORDYN}" -eq 1 && ! -f "${REPO_ROOT}/extern/MoorDyn/CMakeLists.txt" ]]; then
  echo "[FAIL] MoorDyn submodule missing at extern/MoorDyn" >&2
  echo "       Run: git submodule update --init extern/MoorDyn" >&2
  exit 1
fi

if [[ "${DOCTOR}" -eq 1 ]]; then
  echo ""
  echo ">> Doctor: configuration"
  echo "   Config: ${CONFIG_PATH} (exists: $([[ -f "${CONFIG_PATH}" ]] && echo yes || echo no))"
  echo "   ChronoDir (env): ${SEASTACK_CHRONO_DIR:-}"
  echo "   Generator (JSON): ${SEASTACK_CONFIG_GENERATOR:-}"
  echo ""
  echo ">> Doctor: generator"
  if g_try="$(resolve_generator 0)"; then
    echo "   [OK] Resolved: ${g_try}"
    if generator_is_multiconfig "${g_try}"; then
      echo "        Multi-config generator"
    else
      echo "        Single-config generator"
    fi
  else
    echo "   [WARN] Could not resolve generator"
  fi
  echo ""
  echo ">> Doctor: tools"
  command -v cmake >/dev/null 2>&1 && echo "   [OK] cmake: $(command -v cmake)" || echo "   [FAIL] cmake not on PATH"
  command -v ninja >/dev/null 2>&1 && echo "   [OK] ninja: $(command -v ninja)" || echo "   [WARN] ninja not on PATH"
  command -v git >/dev/null 2>&1 && echo "   [OK] git" || echo "   [WARN] git not on PATH"
  echo ""
  echo ">> Doctor: build dir"
  echo "   ${BUILD_DIR} exists: $([[ -d "${BUILD_DIR}" ]] && echo yes || echo no)"
  cg="$(cached_generator || true)"
  [[ -n "${cg}" ]] && echo "   Cached CMAKE_GENERATOR: ${cg}"
  echo ""
  echo "Doctor complete. Log: ${DIAG_LOG}"
  exit 0
fi

if [[ "${CLEAN}" -eq 1 ]]; then
  echo ""
  echo ">> Cleaning"
  removed=()
  if [[ -d "${BUILD_DIR}" ]]; then
    rm -rf "${BUILD_DIR}"
    removed+=("build")
  fi
  shopt -s nullglob
  for d in "${REPO_ROOT}"/build_*; do
    [[ -d "$d" ]] || continue
    rm -rf "$d"
    removed+=("$(basename "$d")")
  done
  shopt -u nullglob
  if [[ "${#removed[@]}" -gt 0 ]]; then
    echo "   [OK] Removed: ${removed[*]}"
  else
    echo "   (no build directories removed)"
  fi
fi

GEN_NAME="$(resolve_generator "${CLEAN}")" || exit 1
echo "   [OK] Generator: ${GEN_NAME}"

if ! command -v cmake >/dev/null 2>&1; then
  echo "[FAIL] cmake not on PATH" >&2
  exit 1
fi
if [[ "${GEN_NAME}" == "Ninja" ]] && ! command -v ninja >/dev/null 2>&1; then
  echo "[FAIL] Generator Ninja selected but ninja not on PATH" >&2
  exit 1
fi

MULTI=0
if generator_is_multiconfig "${GEN_NAME}"; then MULTI=1; fi

mkdir -p "${BUILD_DIR}"

should_configure=1
[[ "${NO_CONFIGURE}" -eq 1 ]] && should_configure=0
[[ "${CONFIGURE_ONLY}" -eq 1 ]] && should_configure=1
if [[ "${NO_CONFIGURE}" -eq 1 ]]; then
  if [[ ! -f "${BUILD_DIR}/CMakeCache.txt" ]]; then
    echo "[FAIL] --no-configure requires ${BUILD_DIR}/CMakeCache.txt" >&2
    exit 1
  fi
fi

seastack_chrono=ON
[[ "${USE_CHRONO}" -eq 1 ]] || seastack_chrono=OFF
seastack_hydro=ON
[[ "${NO_HYDRO_IO}" -eq 1 ]] && seastack_hydro=OFF
seastack_moor=OFF
[[ "${MOORDYN}" -eq 1 ]] && seastack_moor=ON
# Release ZIP ships RM3/OSWEC external_pto demos; packaging implies External ON.
seastack_external=OFF
if [[ "${EXTERNAL}" -eq 1 || "${DO_PACKAGE}" -eq 1 ]]; then
  seastack_external=ON
fi
seastack_vsg=OFF
[[ "${USE_VSG}" -eq 1 ]] && seastack_vsg=ON
seastack_vehicle=OFF
[[ "${VEHICLE}" -eq 1 && "${USE_CHRONO}" -eq 1 ]] && seastack_vehicle=ON
seastack_sph=OFF
[[ "${SPH}" -eq 1 && "${USE_CHRONO}" -eq 1 ]] && seastack_sph=ON
seastack_tests=ON
[[ "${NO_TESTS}" -eq 1 ]] && seastack_tests=OFF
seastack_demos=OFF
[[ "${DEMOS}" -eq 1 ]] && seastack_demos=ON
seastack_apps=ON
[[ "${NO_APPS}" -eq 1 ]] && seastack_apps=OFF

if [[ "${should_configure}" -eq 1 ]]; then
  echo ""
  echo ">> Configuring CMake"
  cmake_args=(
    -S "${REPO_ROOT}"
    -B "${BUILD_DIR}"
    -G "${GEN_NAME}"
    "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}"
    "-DSEASTACK_ENABLE_CHRONO=${seastack_chrono}"
    "-DSEASTACK_ENABLE_HYDRO_IO=${seastack_hydro}"
    "-DSEASTACK_ENABLE_MOORING=${seastack_moor}"
    "-DSEASTACK_ENABLE_EXTERNAL=${seastack_external}"
    "-DSEASTACK_ENABLE_VSG=${seastack_vsg}"
    "-DSEASTACK_ENABLE_VEHICLE=${seastack_vehicle}"
    "-DSEASTACK_ENABLE_SPH=${seastack_sph}"
    "-DSEASTACK_ENABLE_TESTS=${seastack_tests}"
    "-DSEASTACK_ENABLE_DEMOS=${seastack_demos}"
    "-DSEASTACK_ENABLE_APPS=${seastack_apps}"
  )
  if [[ "${USE_CHRONO}" -eq 1 ]]; then
    cmake_args+=("-DChrono_DIR=${SEASTACK_CHRONO_DIR}")
  fi
  if [[ -n "${SEASTACK_EIGEN3_INCLUDE_DIR:-}" ]]; then
    cmake_args+=("-DEIGEN3_INCLUDE_DIR=${SEASTACK_EIGEN3_INCLUDE_DIR}")
  fi
  if [[ -n "${SEASTACK_HDF5_DIR:-}" ]]; then
    cmake_args+=("-DHDF5_DIR=${SEASTACK_HDF5_DIR}")
  fi
  if [[ -n "${SEASTACK_PYTHON_ROOT:-}" && -d "${SEASTACK_PYTHON_ROOT}" ]]; then
    cmake_args+=("-DPython3_ROOT_DIR=${SEASTACK_PYTHON_ROOT}")
  fi

  echo "   Build type : ${BUILD_TYPE}"
  echo "   Generator  : ${GEN_NAME}"
  echo "   Chrono     : ${seastack_chrono}"
  echo "   HydroIO    : ${seastack_hydro}"
  echo "   Mooring    : ${seastack_moor}"
  echo "   VSG        : ${seastack_vsg}"
  echo "   Tests      : ${seastack_tests}"
  echo "   Demos      : ${seastack_demos}"
  echo "   Apps       : ${seastack_apps}"
  log_diag "CMake configure: ${cmake_args[*]}"

  if ! run_captured "CMake configure" cmake "${cmake_args[@]}"; then
    echo "[FAIL] CMake configure failed. Log: ${DIAG_LOG}" >&2
    exit 1
  fi
  echo "   [OK] Configuration complete"
  if [[ "${CONFIGURE_ONLY}" -eq 1 ]]; then
    echo "Configuration written to: ${BUILD_DIR}"
    exit 0
  fi
else
  echo ""
  echo ">> Skipping configure (--no-configure)"
fi

echo ""
echo ">> Building"
build_args=(--build "${BUILD_DIR}" --config "${BUILD_TYPE}")
[[ -n "${TARGET}" ]] && build_args+=(--target "${TARGET}")
log_diag "CMake build: ${build_args[*]}"
if ! run_captured "CMake build" cmake "${build_args[@]}"; then
  echo "[FAIL] Build failed. Log: ${DIAG_LOG}" >&2
  exit 1
fi
echo "   [OK] Build finished"

BIN_SUB="${BUILD_DIR}/bin/${BUILD_TYPE}"
echo ""
echo ">> Verifying outputs"
if [[ -d "${BIN_SUB}" ]]; then
  echo "   Output: ${BIN_SUB}"
  if [[ "${USE_CHRONO}" -eq 1 ]]; then
    if [[ -f "${BIN_SUB}/run_seastack" ]]; then
      echo "   [OK] run_seastack"
    elif [[ -f "${BIN_SUB}/run_seastack.exe" ]]; then
      echo "   [OK] run_seastack.exe"
    fi
  fi
  if [[ -f "${BIN_SUB}/standalone_controller" ]] || [[ -f "${BIN_SUB}/standalone_controller.exe" ]]; then
    echo "   [OK] standalone_controller"
  fi
else
  echo "   [WARN] Expected bin directory not found: ${BIN_SUB}"
fi

if [[ "${DO_TEST}" -eq 1 ]]; then
  echo ""
  echo ">> Running tests"
  ctest_args=(-C "${BUILD_TYPE}" --test-dir "${BUILD_DIR}" --output-on-failure)
  [[ -n "${TEST_LABEL}" ]] && ctest_args+=(-L "${TEST_LABEL}")
  if [[ "${TEST_LABEL}" == "benchmark" ]]; then
    ctest_args+=(-j 1 -E '^benchmark_report_generation$')
  fi
  log_diag "ctest ${ctest_args[*]}"
  set +e
  ctest "${ctest_args[@]}" 2>&1 | tee -a "${DIAG_LOG}"
  ct=$?
  set -e
  log_diag "ctest exit: ${ct}"
  if [[ "${ct}" -ne 0 ]]; then
    echo "   [WARN] Some tests failed (exit ${ct})" >&2
  else
    echo "   [OK] All tests passed"
  fi
  if [[ "${TEST_LABEL}" == "benchmark" ]]; then
    hist="${REPO_ROOT}/data/benchmarks/history"
    rep_py="${REPO_ROOT}/tests/benchmark/utilities/generate_benchmark_report.py"
    out="${BIN_SUB}/results/tests/benchmark/report"
    echo ""
    echo ">> Benchmark report"
    python3 "${rep_py}" --build-dir "${BUILD_DIR}" --output-dir "${out}" --history-dir "${hist}" 2>&1 | tee -a "${DIAG_LOG}" || true
  fi
fi

if [[ "${DO_PACKAGE}" -eq 1 ]]; then
  echo ""
  echo ">> Packaging"
  prefix="${BUILD_DIR}/install"
  if ! run_captured "CMake install" cmake --install "${BUILD_DIR}" --config "${BUILD_TYPE}" --prefix "${prefix}"; then
    echo "[FAIL] cmake --install failed" >&2
    exit 1
  fi
  if ! run_captured "Verify install prefix" python3 "${REPO_ROOT}/scripts/verify_release_install_prefix.py" "${prefix}"; then
    echo "[FAIL] release install prefix verification failed (see scripts/verify_release_install_prefix.py)" >&2
    exit 1
  fi
  if ! run_captured "CPack" bash -c "cd \"${BUILD_DIR}\" && cpack -C \"${BUILD_TYPE}\""; then
    echo "[FAIL] cpack failed" >&2
    exit 1
  fi
  shopt -s nullglob
  pkgs=("${BUILD_DIR}"/SEAStack-*.zip)
  shopt -u nullglob
  pkg=""
  [[ "${#pkgs[@]}" -gt 0 ]] && pkg="${pkgs[0]}"
  if [[ -n "${pkg}" ]]; then
    echo "   [OK] $(basename "${pkg}")"
  fi
fi

echo ""
echo "========================================"
echo "BUILD SUCCESSFUL"
echo "========================================"
echo "Main output: ${BIN_SUB}"
echo ""
if [[ "${NO_TESTS}" -eq 0 ]]; then
  echo "Test suites (from repo root):"
  echo "  ./scripts/unix/ctest_suite.sh unit"
  echo "  ./scripts/unix/ctest_suite.sh chrono-free"
  if [[ "${USE_CHRONO}" -eq 1 ]]; then
    echo "  ./scripts/unix/ctest_suite.sh regression"
    echo "  ./scripts/unix/ctest_suite.sh verification"
    echo "  ./scripts/unix/ctest_suite.sh comparison"
    echo "  ./scripts/unix/ctest_suite.sh benchmark"
  fi
  echo ""
fi
