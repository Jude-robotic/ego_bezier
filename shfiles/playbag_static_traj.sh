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
  ./playbag_static_traj.sh [bag_file]

Examples:
  ./playbag_static_traj.sh
  ./playbag_static_traj.sh bags/swarm_payload_20260422_120227.bag
  ./playbag_static_traj.sh swarm_payload_20260422_120227

Environment overrides:
  BAG_DIR=/path/to/bag_dir
  PLAY_RATE=1.0
  PLAY_START_OFFSET_SEC=0.0
  PLAY_START_DELAY_SEC=1.0
  PLAY_START_PAUSED=0|1
  PLAY_USE_CLOCK=0|1
  RVIZ_CONFIG=/abs/path/to/file.rviz
  RVIZ_STARTUP_WAIT_SEC=2.0
  RVIZ_CLOSE_WITH_PLAYBACK=0|1

Notes:
  - If bag_file is omitted, the newest *.bag in BAG_DIR is used.
  - The script opens RViz and three odom_visualization nodes
    for leader, uav1, and uav2.
  - After rosbag playback ends, RViz stays on the final map frame
    and keeps the full real trajectories visible.
  - Press Ctrl-C to close the visualization session.
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

wait_for_ros_master() {
  local max_attempts="${1:-100}"
  local sleep_sec="${2:-0.2}"
  local attempt

  for (( attempt = 1; attempt <= max_attempts; ++attempt )); do
    if rosparam list >/dev/null 2>&1; then
      return 0
    fi
    if [[ -n "${roslaunch_pid:-}" ]] && ! kill -0 "${roslaunch_pid}" >/dev/null 2>&1; then
      echo "Visualization launch exited before ROS master became ready." >&2
      exit 1
    fi
    sleep "${sleep_sec}"
  done

  echo "Timed out waiting for ROS master to become ready." >&2
  exit 1
}

cleanup() {
  local rc=$?

  set +e

  if [[ -n "${roslaunch_pid:-}" ]]; then
    kill "${roslaunch_pid}" >/dev/null 2>&1 || true
    wait "${roslaunch_pid}" 2>/dev/null || true
  fi

  if [[ -n "${tmp_launch_file:-}" && -f "${tmp_launch_file}" ]]; then
    rm -f "${tmp_launch_file}"
  fi

  exit "${rc}"
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

require_cmd rosbag
require_cmd roslaunch
require_cmd rosparam
require_cmd mktemp

BAG_DIR="${BAG_DIR:-${REPO_ROOT}/bags}"
PLAY_RATE="${PLAY_RATE:-1.0}"
PLAY_START_OFFSET_SEC="${PLAY_START_OFFSET_SEC:-0.0}"
PLAY_START_DELAY_SEC="${PLAY_START_DELAY_SEC:-1.0}"
PLAY_START_PAUSED="${PLAY_START_PAUSED:-0}"
PLAY_USE_CLOCK="${PLAY_USE_CLOCK:-0}"
RVIZ_CONFIG="${RVIZ_CONFIG:-${REPO_ROOT}/src/planner/plan_manage/launch/default.rviz}"
RVIZ_STARTUP_WAIT_SEC="${RVIZ_STARTUP_WAIT_SEC:-2.0}"
RVIZ_CLOSE_WITH_PLAYBACK="${RVIZ_CLOSE_WITH_PLAYBACK:-0}"

if [[ $# -gt 1 ]]; then
  echo "Unexpected extra arguments: ${*:2}" >&2
  usage >&2
  exit 1
fi

if [[ ! -f "${RVIZ_CONFIG}" ]]; then
  echo "RViz config not found: ${RVIZ_CONFIG}" >&2
  exit 1
fi

BAG_PATH="$(resolve_bag_path "${1:-}")"
TOPIC_MANIFEST="${BAG_PATH%.bag}.topics.txt"

if [[ "${PLAY_USE_CLOCK}" == "1" ]]; then
  USE_SIM_TIME="true"
else
  USE_SIM_TIME="false"
fi

tmp_launch_file="$(mktemp "${TMPDIR:-/tmp}/playbag_static_traj_XXXXXX.launch")"
trap cleanup EXIT INT TERM

cat > "${tmp_launch_file}" <<'EOF'
<launch>
  <arg name="rviz_config"/>
  <arg name="use_sim_time" default="false"/>

  <param name="/use_sim_time" value="$(arg use_sim_time)"/>

  <node name="rviz" pkg="rviz" type="rviz" args="-d $(arg rviz_config)" output="screen"/>

  <node pkg="odom_visualization" name="odom_visualization" type="odom_visualization" output="screen">
    <remap from="~odom" to="/visual_slam/odom"/>
    <param name="color/a" value="1.0"/>
    <param name="color/r" value="0.0"/>
    <param name="color/g" value="0.0"/>
    <param name="color/b" value="0.0"/>
    <param name="covariance_scale" value="100.0"/>
    <param name="robot_scale" value="1.0"/>
    <param name="tf45" value="false"/>
  </node>

  <group ns="uav1">
    <node pkg="odom_visualization" name="odom_visualization" type="odom_visualization" output="screen">
      <remap from="~odom" to="/uav1/odom"/>
      <param name="color/a" value="1.0"/>
      <param name="color/r" value="0.1"/>
      <param name="color/g" value="0.6"/>
      <param name="color/b" value="1.0"/>
      <param name="covariance_scale" value="100.0"/>
      <param name="robot_scale" value="0.8"/>
      <param name="tf45" value="false"/>
    </node>
  </group>

  <group ns="uav2">
    <node pkg="odom_visualization" name="odom_visualization" type="odom_visualization" output="screen">
      <remap from="~odom" to="/uav2/odom"/>
      <param name="color/a" value="1.0"/>
      <param name="color/r" value="1.0"/>
      <param name="color/g" value="0.45"/>
      <param name="color/b" value="0.1"/>
      <param name="covariance_scale" value="100.0"/>
      <param name="robot_scale" value="0.8"/>
      <param name="tf45" value="false"/>
    </node>
  </group>
</launch>
EOF

launch_cmd=(
  roslaunch
  "${tmp_launch_file}"
  "rviz_config:=${RVIZ_CONFIG}"
  "use_sim_time:=${USE_SIM_TIME}"
)

cmd=(
  rosbag play
  "${BAG_PATH}"
  -r "${PLAY_RATE}"
  -s "${PLAY_START_OFFSET_SEC}"
  --delay="${PLAY_START_DELAY_SEC}"
)

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
printf 'Use /clock: %s\n' "${PLAY_USE_CLOCK}"
printf 'RViz config: %s\n' "${RVIZ_CONFIG}"

if [[ -f "${TOPIC_MANIFEST}" ]]; then
  printf 'Topic manifest: %s\n' "${TOPIC_MANIFEST}"
fi

printf 'Trajectory sources: /visual_slam/odom, /uav1/odom, /uav2/odom\n'
printf 'Static view behavior: keep RViz on the final map frame after playback.\n'
printf 'rosbag info:\n'
rosbag info "${BAG_PATH}"

printf 'Starting visualization launch:'
printf ' %q' "${launch_cmd[@]}"
printf '\n'
"${launch_cmd[@]}" &
roslaunch_pid=$!

wait_for_ros_master

if [[ "${RVIZ_STARTUP_WAIT_SEC}" != "0" ]]; then
  printf 'Waiting %s s for RViz/visualizers startup before playback...\n' "${RVIZ_STARTUP_WAIT_SEC}"
  sleep "${RVIZ_STARTUP_WAIT_SEC}"
fi

printf 'Playback command:'
printf ' %q' "${cmd[@]}"
printf '\n'

set +e
"${cmd[@]}"
play_rc=$?
set -e

if [[ "${play_rc}" -ne 0 ]]; then
  exit "${play_rc}"
fi

if [[ "${RVIZ_CLOSE_WITH_PLAYBACK}" == "1" ]]; then
  printf 'Playback finished. Closing RViz and visualization nodes.\n'
  exit 0
fi

printf 'Playback finished. RViz is holding the final map frame and full real trajectories.\n'
printf 'Press Ctrl-C to close the visualization session.\n'

wait "${roslaunch_pid}"
