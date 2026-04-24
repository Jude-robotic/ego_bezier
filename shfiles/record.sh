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
  ./record.sh [payload|nonehall] [bag_name_or_path]

Examples:
  ./record.sh
  ./record.sh payload
  ./record.sh nonehall door_slit_ring_check
  ./record.sh payload /tmp/swarm_payload_check.bag

Environment overrides:
  BAG_DIR=/path/to/output_dir
  BAG_PREFIX=custom_prefix
  BAG_PROFILE=full|lite
  RECORD_APPEND_TIMESTAMP=1|0
  TIMESTAMP_FORMAT=%Y%m%d_%H%M%S
  RECORD_EXTRA_TOPICS="/foo /bar"

Notes:
  - full: record runtime chain + leader sensor/map inputs, suitable for replay/analysis.
  - lite: record runtime chain only, omits high-bandwidth map/depth topics.
  - The bag includes /swarm/formation_mode, so your door/slit/ring switch commands
    are replayable directly from the recorded bag.
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

build_bag_path() {
  local raw_path="$1"
  local timestamp=""
  local stem
  local ext

  if [[ "${RECORD_APPEND_TIMESTAMP}" == "1" ]]; then
    timestamp="_$(date +"${TIMESTAMP_FORMAT}")"
  fi

  if [[ -z "${raw_path}" ]]; then
    raw_path="${BAG_DIR}/${BAG_PREFIX}"
  elif [[ "${raw_path}" != /* ]]; then
    raw_path="${BAG_DIR}/${raw_path}"
  fi

  if [[ "${raw_path}" == *.bag ]]; then
    stem="${raw_path%.bag}"
    ext=".bag"
  else
    stem="${raw_path}"
    ext=".bag"
  fi

  abs_path "${stem}${timestamp}${ext}"
}

dedupe_topics() {
  declare -A seen=()
  deduped_topics=()

  local topic
  for topic in "${topics[@]}"; do
    if [[ -n "${seen[${topic}]:-}" ]]; then
      continue
    fi
    seen["${topic}"]=1
    deduped_topics+=("${topic}")
  done
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

require_cmd rosbag

SCENE="payload"
if [[ "${1:-}" == "payload" || "${1:-}" == "nonehall" ]]; then
  SCENE="$1"
  shift
fi

BAG_ARG="${1:-}"
if [[ $# -gt 1 ]]; then
  echo "Unexpected extra arguments: ${*:2}" >&2
  usage >&2
  exit 1
fi

BAG_DIR="${BAG_DIR:-${REPO_ROOT}/bags}"
BAG_PREFIX="${BAG_PREFIX:-swarm_${SCENE}}"
BAG_PROFILE="${BAG_PROFILE:-full}"
RECORD_APPEND_TIMESTAMP="${RECORD_APPEND_TIMESTAMP:-1}"
TIMESTAMP_FORMAT="${TIMESTAMP_FORMAT:-%Y%m%d_%H%M%S}"

case "${BAG_PROFILE}" in
  full|lite)
    ;;
  *)
    echo "Unsupported BAG_PROFILE=${BAG_PROFILE}. Expected full or lite." >&2
    exit 1
    ;;
esac

declare -a topics=(
  "/tf"
  "/tf_static"
  "/move_base_simple/goal"
  "/traj_start_trigger"
  "/swarm/traj_start_trigger_gated"
  "/swarm/preflight/ok"
  "/swarm/formation_mode"
  "/swarm/startup_sync/release"
  "/swarm/agent_1/startup_ready"
  "/swarm/agent_2/startup_ready"
  "/visual_slam/odom"
  "/planning/bezier"
  "/planning/pos_cmd"
  "/swarm/leader/corrected_bezier"
  "/swarm/master/agent_1/guidance_bezier"
  "/swarm/master/agent_2/guidance_bezier"
  "/swarm/program_ring/agent_1/guidance_bezier"
  "/swarm/program_ring/agent_2/guidance_bezier"
  "/swarm/program_narrow/agent_1/guidance_bezier"
  "/swarm/program_narrow/agent_2/guidance_bezier"
  "/swarm/program_door/agent_1/guidance_bezier"
  "/swarm/program_door/agent_2/guidance_bezier"
  "/swarm/program_slit/agent_1/guidance_bezier"
  "/swarm/program_slit/agent_2/guidance_bezier"
  "/swarm/agent_1/guidance_bezier"
  "/swarm/agent_2/guidance_bezier"
  "/swarm/agent_1/state"
  "/swarm/agent_2/state"
  "/uav1/odom"
  "/uav2/odom"
  "/uav1/planning/bezier"
  "/uav2/planning/bezier"
  "/uav1/planning/pos_cmd"
  "/uav2/planning/pos_cmd"
  "/swarm/comm/markers"
  "/swarm/guidance_viz/markers"
  "/swarm/payload_viz/markers"
  "/pcl_render_node/camera_pose"
)

if [[ "${BAG_PROFILE}" == "full" ]]; then
  topics+=(
    "/map_generator/global_cloud"
    "/pcl_render_node/cloud"
    "/pcl_render_node/depth"
  )
fi

if [[ -n "${RECORD_EXTRA_TOPICS:-}" ]]; then
  read -r -a extra_topics <<< "${RECORD_EXTRA_TOPICS}"
  topics+=("${extra_topics[@]}")
fi

dedupe_topics

BAG_PATH="$(build_bag_path "${BAG_ARG}")"
mkdir -p "$(dirname "${BAG_PATH}")"

if [[ -e "${BAG_PATH}" ]]; then
  echo "Bag already exists: ${BAG_PATH}" >&2
  exit 1
fi

TOPIC_MANIFEST="${BAG_PATH%.bag}.topics.txt"
{
  printf '# scene=%s\n' "${SCENE}"
  printf '# profile=%s\n' "${BAG_PROFILE}"
  printf '# generated_at=%s\n' "$(date --iso-8601=seconds)"
  printf '# chain=leader_planner -> swarm_master -> program_* -> guidance_mux -> followers -> visualizers\n'
  printf '%s\n' "${deduped_topics[@]}"
} > "${TOPIC_MANIFEST}"

printf 'Scene: %s\n' "${SCENE}"
printf 'Profile: %s\n' "${BAG_PROFILE}"
printf 'Output bag: %s\n' "${BAG_PATH}"
printf 'Topic manifest: %s\n' "${TOPIC_MANIFEST}"
printf 'Topic count: %d\n' "${#deduped_topics[@]}"
printf 'Recorded formation commands include: program_ring, program_door, program_slit, normal.\n'
printf 'Stop recording with Ctrl-C.\n'

exec rosbag record -O "${BAG_PATH}" "${deduped_topics[@]}"
