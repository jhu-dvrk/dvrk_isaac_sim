#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ENVIRONMENTS_DIR="${SCRIPT_DIR}/../share/environments"
ENVIRONMENT="test_cube"

usage() {
  echo "Usage: $0 [--env NAME]"
  echo "Select an environment YAML under share/environments. Default: test_cube"
  echo "Available environments:"
  for environment_file in "${ENVIRONMENTS_DIR}"/*.yaml; do
    [[ -e "${environment_file}" ]] || continue
    printf '  %s\n' "$(basename "${environment_file}" .yaml)"
  done
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --env|-e)
      if [[ $# -lt 2 || -z "$2" ]]; then
        echo "Option $1 requires an environment name." >&2
        usage >&2
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

export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ISAAC_SIM_DIR="${HOME}/isaacsim"
export DVRK_MODEL_PATH="${HOME}/dvrk_ws/src/dvrk/dvrk_model"

unset AMENT_TRACE_SETUP_FILES

if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi

if [ -f "${HOME}/dvrk_ws/install/setup.bash" ]; then
  source "${HOME}/dvrk_ws/install/setup.bash"
fi

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "gnome-terminal is required to open the split dVRK terminals." >&2
  exit 1
fi

# Clean up stale sim processes before launching.  A lingering Isaac Sim session
# often preserves old USD stage state and triggers the startup wait-timeout/crash
# condition when a new launch tries to reuse the previous process.
for pattern in \
  "isaacsim" \
  "python.sh.*simulator.py" \
  "dvrk_system" \
  "ros2.*launch.*dvrk_isaac_sim"; do
  pkill -f "$pattern" || true
done

ros2 daemon stop || true

# Give the OS a moment to cleanly terminate any stale processes before the new
# launch starts.  This avoids reusing a dead/half-initialized Isaac stage.
for _ in 1 2 3; do
  if ! pgrep -f "isaacsim|python.sh.*simulator.py|dvrk_system" >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

gnome-terminal --title="Isaac Sim" -- bash -lc '
  set -eo pipefail
  unset AMENT_TRACE_SETUP_FILES
  source /opt/ros/jazzy/setup.bash || true
  source "$HOME/dvrk_ws/install/setup.bash" || true
  export ROS_DISTRO=jazzy
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ISAAC_SIM_DIR="$HOME/isaacsim"
  export DVRK_MODEL_PATH="$HOME/dvrk_ws/src/dvrk/dvrk_model"
  cd "$HOME/dvrk_ws"
  ros2 launch dvrk_isaac_sim simulator.launch.py scene:=ECM_PSM1_PSM2_PSM3_mono.yaml env:="$1"
' _ "$ENVIRONMENT"

gnome-terminal --title="Haply System" -- bash -lc '
  set -eo pipefail
  unset AMENT_TRACE_SETUP_FILES
  source /opt/ros/jazzy/setup.bash || true
  source "$HOME/dvrk_ws/install/setup.bash" || true
  export ROS_DISTRO=jazzy
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ISAAC_SIM_DIR="$HOME/isaacsim"
  export DVRK_MODEL_PATH="$HOME/dvrk_ws/src/dvrk/dvrk_model"
  cd "$HOME/dvrk_ws/src/dvrk/dvrk_isaac_sim/share/dvrk_systems"
  ros2 run dvrk_robot dvrk_system -j "$HOME/dvrk_ws/src/dvrk/dvrk_isaac_sim/share/dvrk_systems/system-MTMR-PSM1-Haply-ROS.json"
'

echo "Opened Isaac Sim (environment: ${ENVIRONMENT}) and Haply terminals after cleaning stale processes."
