#!/usr/bin/env bash
# Downstream-consumer check (macOS / Linux): install a Chrono-free Debug
# SEA-Stack SDK, build tests/consumer against it with find_package(SEAStack),
# and run it. Guards the exported CMake package and usage requirements.
#
# Usage: scripts/unix/run_consumer_check.sh [--work-dir DIR] [--prefix-path PATH]
#   --prefix-path  extra CMAKE_PREFIX_PATH for Eigen3/HDF5 (e.g. /opt/homebrew)
# HDF5Dir from build-config.json is passed as HDF5_DIR when set.
# Work files go to a new directory under ${TMPDIR:-/tmp} unless --work-dir is given.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WORK_DIR=""
PREFIX_PATH=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --work-dir) WORK_DIR="$2"; shift 2 ;;
    --prefix-path) PREFIX_PATH="$2"; shift 2 ;;
    -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
    *) echo "[FAIL] unknown argument: $1" >&2; exit 2 ;;
  esac
done

fail() { echo "[FAIL] $*" >&2; exit 1; }

command -v cmake >/dev/null || fail "cmake not on PATH"
GEN_ARGS=()
if command -v ninja >/dev/null; then GEN_ARGS=(-G Ninja); fi

HDF5_DIR_ARG=()
CONFIG_PATH="${REPO_ROOT}/build-config.json"
if [[ -f "${CONFIG_PATH}" ]]; then
  hdf5_dir="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("HDF5Dir",""))' "${CONFIG_PATH}")"
  if [[ -n "${hdf5_dir}" ]]; then HDF5_DIR_ARG=("-DHDF5_DIR=${hdf5_dir}"); fi
fi

if [[ -z "${WORK_DIR}" ]]; then
  WORK_DIR="${TMPDIR:-/tmp}/seastack-consumer-check-$(date +%Y%m%d-%H%M%S)"
fi
[[ -e "${WORK_DIR}" ]] && fail "work dir already exists: ${WORK_DIR}"
mkdir -p "${WORK_DIR}"
SDK_BUILD="${WORK_DIR}/sdk-build"
SDK="${WORK_DIR}/sdk"
CONSUMER_BUILD="${WORK_DIR}/consumer-build"
H5="${REPO_ROOT}/data/demos/run_seastack/rm3/assets/hydroData/rm3.h5"

echo "Repo:        ${REPO_ROOT}"
echo "Work dir:    ${WORK_DIR}"
echo "Prefix path: ${PREFIX_PATH}"

PREFIX_ARG=()
[[ -n "${PREFIX_PATH}" ]] && PREFIX_ARG=("-DCMAKE_PREFIX_PATH=${PREFIX_PATH}")

echo ">> Configure + install Chrono-free Debug SDK"
cmake -S "${REPO_ROOT}" -B "${SDK_BUILD}" ${GEN_ARGS[@]+"${GEN_ARGS[@]}"} -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_INSTALL_PREFIX="${SDK}" -DSEASTACK_ENABLE_CHRONO=OFF -DSEASTACK_ENABLE_APPS=OFF \
  -DSEASTACK_ENABLE_TESTS=OFF -DSEASTACK_ENABLE_HYDRO_IO=ON \
  ${HDF5_DIR_ARG[@]+"${HDF5_DIR_ARG[@]}"} ${PREFIX_ARG[@]+"${PREFIX_ARG[@]}"} | tail -n 1 || fail "SDK configure"
cmake --build "${SDK_BUILD}" --parallel | tail -n 1 || fail "SDK build"
cmake --install "${SDK_BUILD}" >/dev/null || fail "SDK install"

echo ">> Configure + build consumer (Debug) against the installed SDK"
CONSUMER_PREFIX="${SDK}"
[[ -n "${PREFIX_PATH}" ]] && CONSUMER_PREFIX="${SDK};${PREFIX_PATH}"
cmake -S "${REPO_ROOT}/tests/consumer" -B "${CONSUMER_BUILD}" ${GEN_ARGS[@]+"${GEN_ARGS[@]}"} \
  -DCMAKE_BUILD_TYPE=Debug "-DCMAKE_PREFIX_PATH=${CONSUMER_PREFIX}" ${HDF5_DIR_ARG[@]+"${HDF5_DIR_ARG[@]}"} \
  | tail -n 1 || fail "consumer configure"
cmake --build "${CONSUMER_BUILD}" | tail -n 1 || fail "consumer build"

echo ">> Run consumer"
set +e
output="$("${CONSUMER_BUILD}/consumer_check" "${H5}" 2>&1)"
code=$?
set -e
echo "${output}" | sed 's/^/   /'
if [[ ${code} -ne 0 ]] || ! grep -q "consumer OK" <<<"${output}"; then
  fail "consumer run exit code ${code}"
fi
echo "[OK] Downstream-consumer check passed (work files: ${WORK_DIR})"
