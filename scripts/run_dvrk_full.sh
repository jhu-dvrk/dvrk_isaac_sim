#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../../../.." && pwd)"

# Do not inherit another workspace or a login shell's automatic ROS setup.
if [[ "${DVRK_FULL_CLEAN:-}" != "${WORKSPACE_DIR}" ]]; then
  exec env -i \
    HOME="${HOME}" USER="${USER:-}" LOGNAME="${LOGNAME:-}" \
    PATH=/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin \
    LANG="${LANG:-C.UTF-8}" TERM="${TERM:-xterm-256color}" \
    DISPLAY="${DISPLAY:-}" WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" \
    XAUTHORITY="${XAUTHORITY:-}" XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" \
    DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-}" \
    ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-${HOME}/isaacsim}" \
    DVRK_COLLISION_PYTHON="${WORKSPACE_DIR}/.venv-newton/bin/python" \
    DVRK_FULL_CLEAN="${WORKSPACE_DIR}" \
    bash --noprofile --norc "${BASH_SOURCE[0]}" "$@"
fi

ENVIRONMENTS_DIR="${SCRIPT_DIR}/../share/environments"
SCENES_DIR="${SCRIPT_DIR}/../share/scenes"
ENVIRONMENT="test_cube"
SCENE="ECM_PSM1_PSM2_PSM3_mono.yaml"
SYSTEM_CONFIG="${SCRIPT_DIR}/../share/dvrk_systems/system-MTMR-PSM1-Haply-ROS.json"
CHECK_ONLY=false
TERMINAL_ROLE=""

usage() {
  echo "Usage: $0 [--env NAME] [--scene NAME] [--system-config PATH] [--check]"
  echo "Launch this workspace's Isaac Sim + Newton collisions and Haply."
  echo "Default robot scene: ${SCENE}"
  echo "Default environment: test_cube"
  echo "Available environments:"
  for environment_file in "${ENVIRONMENTS_DIR}"/*.yaml; do
    [[ -e "${environment_file}" ]] || continue
    printf '  %s\n' "$(basename "${environment_file}" .yaml)"
  done
  echo "--check validates setup without starting the simulator or Haply."
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --env|-e|--scene|-s|--system-config|--_terminal)
      if [[ $# -lt 2 || -z "$2" || "$2" == --* ]]; then
        echo "Option $1 requires a value." >&2
        exit 2
      fi
      case "$1" in
        --env|-e) ENVIRONMENT="$2" ;;
        --scene|-s) SCENE="$2" ;;
        --system-config) SYSTEM_CONFIG="$2" ;;
        --_terminal) TERMINAL_ROLE="$2" ;;
      esac
      shift 2
      ;;
    --check) CHECK_ONLY=true; shift ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

resolve_yaml() {
  local selection="$1" directory="$2"
  if [[ "${selection}" != */* ]]; then
    selection="${directory}/${selection}"
  fi
  if [[ ! -f "${selection}" && -f "${selection}.yaml" ]]; then
    selection="${selection}.yaml"
  fi
  if [[ ! -f "${selection}" ]]; then
    echo "YAML file not found: ${selection}" >&2
    return 2
  fi
  realpath -- "${selection}"
}

ENVIRONMENT="$(resolve_yaml "${ENVIRONMENT}" "${ENVIRONMENTS_DIR}")"
SCENE="$(resolve_yaml "${SCENE}" "${SCENES_DIR}")"
SYSTEM_CONFIG="$(realpath -e -- "${SYSTEM_CONFIG}")"

setup_workspace() {
  if [[ ! -f "${WORKSPACE_DIR}/install/local_setup.bash" ]]; then
    echo "Build this workspace first: ${WORKSPACE_DIR}" >&2
    return 1
  fi
  source /opt/ros/jazzy/setup.bash || return
  source "${WORKSPACE_DIR}/install/local_setup.bash" || return
  export ROS_DISTRO=jazzy RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export DVRK_MODEL_PATH="${WORKSPACE_DIR}/src/dvrk/dvrk_model"
  export DVRK_NEWTON_PYTHON="${DVRK_COLLISION_PYTHON}"
}

run_terminal() {
  setup_workspace || return
  cd "${WORKSPACE_DIR}" || return
  case "${TERMINAL_ROLE}" in
    simulator)
      ros2 launch dvrk_isaac_sim simulator.launch.py \
        scene:="${SCENE}" env:="${ENVIRONMENT}" collision_backend:=newton \
        isaac_sim_dir:="${ISAAC_SIM_DIR}"
      ;;
    haply)
      cd -- "$(dirname -- "${SYSTEM_CONFIG}")" || return
      ros2 run dvrk_robot dvrk_system -j "${SYSTEM_CONFIG}"
      ;;
    *) echo "Unknown terminal role: ${TERMINAL_ROLE}" >&2; return 2 ;;
  esac
}

if [[ -n "${TERMINAL_ROLE}" ]]; then
  set +e
  run_terminal
  status=$?
  printf '\n%s exited with status %s. Press Enter to close this terminal.\n' \
    "${TERMINAL_ROLE}" "${status}"
  read -r _
  exit "${status}"
fi

setup_workspace

for package in dvrk_isaac_sim dvrk_robot dvrk_model; do
  prefix="$(ros2 pkg prefix "${package}")"
  if [[ "${prefix}" != "${WORKSPACE_DIR}/install/"* ]]; then
    echo "${package} resolves outside this workspace: ${prefix}" >&2
    exit 1
  fi
done

isaac_prefix="$(ros2 pkg prefix dvrk_isaac_sim)"
python3 "${isaac_prefix}/share/dvrk_isaac_sim/scripts/_isaac_sim_build.py"

if [[ ! -x "${ISAAC_SIM_DIR}/python.sh" ]]; then
  echo "Isaac Sim python.sh not found in ${ISAAC_SIM_DIR}." >&2
  exit 1
fi
if [[ ! -x "${DVRK_COLLISION_PYTHON}" ]]; then
  echo "Newton Python not found: ${DVRK_COLLISION_PYTHON}" >&2
  echo "Create the local collision environment as described in README.md." >&2
  exit 1
fi
if [[ ! -f "${WORKSPACE_DIR}/install/cisst/lib/libsawHaplyService.so" ]]; then
  echo "Build sawHaplyServiceCore in this workspace before launching Haply." >&2
  exit 1
fi

python3 -c '
import sys
from dvrk_isaac_sim.scene import load_scene, merge_scene_environment
merge_scene_environment(load_scene(sys.argv[1]), load_scene(sys.argv[2]))
' "${SCENE}" "${ENVIRONMENT}"
"${DVRK_COLLISION_PYTHON}" -c 'import newton, warp, pxr, trimesh, vhacdx'

if "${CHECK_ONLY}"; then
  echo "Setup verified: ${WORKSPACE_DIR}"
  echo "Scene: ${SCENE}"
  echo "Environment: ${ENVIRONMENT}"
  echo "Haply config: ${SYSTEM_CONFIG}"
  exit 0
fi

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "gnome-terminal is required to open the dVRK terminals." >&2
  exit 1
fi

for role in simulator haply; do
  # Reset the marker: GNOME's existing server may have a different environment.
  gnome-terminal --title="dvrk_newton_ws: ${role}" -- \
    env -u DVRK_FULL_CLEAN \
    ISAAC_SIM_DIR="${ISAAC_SIM_DIR}" DVRK_COLLISION_PYTHON="${DVRK_COLLISION_PYTHON}" \
    ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
    bash --noprofile --norc "${SCRIPT_DIR}/run_dvrk_full.sh" \
      --_terminal "${role}" --scene "${SCENE}" --env "${ENVIRONMENT}" \
      --system-config "${SYSTEM_CONFIG}"
done

echo "Opened this workspace's Isaac Sim + Newton and Haply terminals."
echo "Startup errors remain visible in their terminals."
