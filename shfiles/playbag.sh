#!/usr/bin/env bash

if [ -z "${BASH_VERSION:-}" ]; then
  exec bash "$0" "$@"
fi

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP_SCRIPT="${REPO_ROOT}/devel/setup.bash"

if [[ -f "${SETUP_SCRIPT}" ]]; then
  setup_args=("$@")
  set --
  # shellcheck source=/dev/null
  source "${SETUP_SCRIPT}"
  set -- "${setup_args[@]}"
  unset setup_args
fi

usage() {
  cat <<'EOF'
Usage:
  shfiles/playbag.sh [bag_file]

Examples:
  shfiles/playbag.sh
  shfiles/playbag.sh bags/swarm_payload_20260422_120000.bag
  shfiles/playbag.sh swarm_payload_20260422_120000

Environment overrides:
  BAG_DIR=/path/to/bag_dir
  PLAY_RATE=1.0
  PLAY_START_OFFSET_SEC=0.0
  PLAY_START_DELAY_SEC=1.0
  PLAY_LOOP=0|1
  PLAY_START_PAUSED=0|1
  PLAY_USE_CLOCK=0|1
  RVIZ_AUTO_OPEN=1|0
  RVIZ_CONFIG=/abs/path/to/file.rviz
  RVIZ_EXECUTABLE=rviz
  RVIZ_STARTUP_WAIT_SEC=2.0
  RVIZ_CLOSE_WITH_PLAYBACK=0|1

Notes:
  - If bag_file is omitted, the newest *.bag in BAG_DIR is used.
  - The playback start delay is parameterized through PLAY_START_DELAY_SEC.
  - When RVIZ_AUTO_OPEN=1, the script opens RViz first, then waits
    RVIZ_STARTUP_WAIT_SEC before starting rosbag playback.
EOF
}

require_cmd() {
  local name="$1"
  if ! command -v "${name}" >/dev/null 2>&1; then
    echo "Missing required command: ${name}" >&2
    exit 1
  fi
}

abs_path() {
  local path="$1"
  if [[ "${path}" = /* ]]; then
    printf '%s\n' "${path}"
    return
  fi

  printf '%s/%s\n' "${PWD}" "${path}"
}

find_latest_bag() {
  local bag_dir="$1"
  if [[ ! -d "${bag_dir}" ]]; then
    return 1
  fi

  find "${bag_dir}" -maxdepth 1 -type f -name '*.bag' -printf '%T@ %p\n' \
    | sort -nr \
    | head -n 1 \
    | cut -d' ' -f2-
}

resolve_bag_path() {
  local raw="$1"
  local candidate
  local -a candidates=()

  if [[ -z "${raw}" ]]; then
    candidate="$(find_latest_bag "${BAG_DIR}" || true)"
    if [[ -z "${candidate}" ]]; then
      echo "No .bag file found in ${BAG_DIR}" >&2
      exit 1
    fi
    abs_path "${candidate}"
    return
  fi

  candidates+=("${raw}")
  if [[ "${raw}" != *.bag ]]; then
    candidates+=("${raw}.bag")
  fi

  if [[ "${raw}" != /* ]]; then
    candidates+=("${BAG_DIR}/${raw}")
    if [[ "${raw}" != *.bag ]]; then
      candidates+=("${BAG_DIR}/${raw}.bag")
    fi
  fi

  for candidate in "${candidates[@]}"; do
    if [[ -f "${candidate}" ]]; then
      abs_path "${candidate}"
      return
    fi
  done

  echo "Cannot find bag file from input: ${raw}" >&2
  exit 1
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

require_cmd rosbag

BAG_DIR="${BAG_DIR:-${REPO_ROOT}/bags}"
PLAY_RATE="${PLAY_RATE:-1.0}"
PLAY_START_OFFSET_SEC="${PLAY_START_OFFSET_SEC:-0.0}"
PLAY_START_DELAY_SEC="${PLAY_START_DELAY_SEC:-1.0}"
PLAY_LOOP="${PLAY_LOOP:-0}"
PLAY_START_PAUSED="${PLAY_START_PAUSED:-0}"
PLAY_USE_CLOCK="${PLAY_USE_CLOCK:-0}"
RVIZ_AUTO_OPEN="${RVIZ_AUTO_OPEN:-1}"
RVIZ_CONFIG="${RVIZ_CONFIG:-${REPO_ROOT}/src/planner/plan_manage/launch/default.rviz}"
RVIZ_EXECUTABLE="${RVIZ_EXECUTABLE:-rviz}"
RVIZ_STARTUP_WAIT_SEC="${RVIZ_STARTUP_WAIT_SEC:-2.0}"
RVIZ_CLOSE_WITH_PLAYBACK="${RVIZ_CLOSE_WITH_PLAYBACK:-0}"

if [[ $# -gt 1 ]]; then
  echo "Unexpected extra arguments: ${*:2}" >&2
  usage >&2
  exit 1
fi

BAG_PATH="$(resolve_bag_path "${1:-}")"
TOPIC_MANIFEST="${BAG_PATH%.bag}.topics.txt"
rviz_pid=""

cmd=(rosbag play "${BAG_PATH}" -r "${PLAY_RATE}" -s "${PLAY_START_OFFSET_SEC}" --delay="${PLAY_START_DELAY_SEC}")

if [[ "${PLAY_LOOP}" == "1" ]]; then
  cmd+=(-l)
fi

if [[ "${PLAY_START_PAUSED}" == "1" ]]; then
  cmd+=(--pause)
fi

if [[ "${PLAY_USE_CLOCK}" == "1" ]]; then
  cmd+=(--clock)
fi

printf 'Bag: %s\n' "${BAG_PATH}"
printf 'Playback rate: %s\n' "${PLAY_RATE}"
printf 'Playback offset: %s s\n' "${PLAY_START_OFFSET_SEC}"
printf 'Playback start delay: %s s\n' "${PLAY_START_DELAY_SEC}"
printf 'RViz auto open: %s\n' "${RVIZ_AUTO_OPEN}"

if [[ -f "${TOPIC_MANIFEST}" ]]; then
  printf 'Topic manifest: %s\n' "${TOPIC_MANIFEST}"
fi

printf 'Useful review topics: /swarm/comm/markers, /swarm/guidance_viz/markers, /swarm/payload_viz/markers\n'
printf 'rosbag info:\n'
rosbag info "${BAG_PATH}"

if [[ "${RVIZ_AUTO_OPEN}" == "1" ]]; then
  require_cmd "${RVIZ_EXECUTABLE}"

  if [[ ! -f "${RVIZ_CONFIG}" ]]; then
    echo "RViz config not found: ${RVIZ_CONFIG}" >&2
    exit 1
  fi

  printf 'Starting RViz: %s -d %s\n' "${RVIZ_EXECUTABLE}" "${RVIZ_CONFIG}"
  "${RVIZ_EXECUTABLE}" -d "${RVIZ_CONFIG}" &
  rviz_pid=$!
  printf 'RViz PID: %s\n' "${rviz_pid}"

  if [[ "${RVIZ_STARTUP_WAIT_SEC}" != "0" ]]; then
    printf 'Waiting %s s for RViz startup before playback...\n' "${RVIZ_STARTUP_WAIT_SEC}"
    sleep "${RVIZ_STARTUP_WAIT_SEC}"
  fi
fi

printf 'Command:'
printf ' %q' "${cmd[@]}"
printf '\n'

set +e
"${cmd[@]}"
play_rc=$?
set -e

if [[ -n "${rviz_pid}" && "${RVIZ_CLOSE_WITH_PLAYBACK}" == "1" ]]; then
  printf 'Closing RViz PID %s\n' "${rviz_pid}"
  kill "${rviz_pid}" >/dev/null 2>&1 || true
  wait "${rviz_pid}" 2>/dev/null || true
fi

exit "${play_rc}"
