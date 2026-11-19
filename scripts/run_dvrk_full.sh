#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd -- "${SCRIPT_DIR}/../../../.." && pwd)"
SCENES_DIR="${SCRIPT_DIR}/../share/scenes"
ENVIRONMENT="test_cube"

usage() {
  echo "Usage: $0 [--env NAME_OR_PATH]"
  echo "Start daVinci-Si Isaac Sim, physical MTMs, MTML DQLA pedals, and the control panel."
  echo "Default exercise: test_cube. Homing and teleoperation remain manual."
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --env|-e)
      if [[ $# -lt 2 || -z "$2" ]]; then
        echo "Option $1 requires an exercise name or YAML path." >&2
        exit 2
      fi
      ENVIRONMENT="$2"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "${ENVIRONMENT}" != */* ]]; then
  case "${ENVIRONMENT%.yaml}" in
    test_cube|ball_tray|match_row)
      ENVIRONMENT="${ENVIRONMENT%.yaml}_objects.yaml"
      ;;
  esac
  if [[ -f "${SCENES_DIR}/${ENVIRONMENT}" ]]; then
    ENVIRONMENT="${SCENES_DIR}/${ENVIRONMENT}"
  else
    ENVIRONMENT="${SCENES_DIR}/${ENVIRONMENT}.yaml"
  fi
fi
if [[ ! -f "${ENVIRONMENT}" ]]; then
  echo "Exercise file not found: ${ENVIRONMENT}" >&2
  exit 2
fi
ENVIRONMENT="$(realpath -- "${ENVIRONMENT}")"

for setup_file in /opt/ros/jazzy/setup.bash "${WORKSPACE}/install/setup.bash"; do
  if [[ ! -f "${setup_file}" ]]; then
    echo "Required setup file not found: ${setup_file}" >&2
    exit 1
  fi
done

unset AMENT_TRACE_SETUP_FILES
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE}/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

cd "${WORKSPACE}"
exec ros2 launch dvrk_config_jhu davinci_si_isaac_sim.launch.py exercise:="${ENVIRONMENT}"
