#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Start FAST-LIO2 SLAM + competition navigation + PCD saving.

FAST-LIO2 lives in the fast_lio2 package (ws_livox) and is driven by the
fastlio2_merge integration package (SimEnv).  Sourcing order matters: SimEnv
and ws_livox are two independent catkin workspaces whose setup.bash files
overwrite each other's ROS_PACKAGE_PATH.  This script sources both and then
merges their src paths explicitly.

Usage:
  start_slam_navigation_fastlio2.sh [--simenv PATH] [--wslivox PATH] [-- roslaunch-arg:=value ...]

Options:
  --simenv PATH     SimEnv workspace (default: $HOME/SimEnv)
  --wslivox PATH    ws_livox workspace containing FAST-LIO2 (default: $HOME/ws_livox)
  -h, --help        Show this help

The Gazebo competition environment and junior_ctrl must already be running.
Use -- to pass additional arguments to slam_nav.launch.
EOF
}

die() {
  echo "start_slam_navigation_fastlio2: $*" >&2
  exit 2
}

SIMENV="${SIMENV:-$HOME/SimEnv}"
WSLIVOX="${WSLIVOX:-$HOME/ws_livox}"
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --simenv)
      [[ $# -ge 2 ]] || die "--simenv requires a path"
      SIMENV="$2"
      shift 2
      ;;
    --wslivox)
      [[ $# -ge 2 ]] || die "--wslivox requires a path"
      WSLIVOX="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      EXTRA_ARGS+=("$@")
      break
      ;;
    *)
      die "unknown option: $1 (use --help)"
      ;;
  esac
done

[[ -f "$SIMENV/devel/setup.bash" ]] || die "missing $SIMENV/devel/setup.bash; build SimEnv first"
[[ -f "$WSLIVOX/devel/setup.bash" ]] || die "missing $WSLIVOX/devel/setup.bash; build ws_livox first"
[[ -f /opt/ros/noetic/setup.bash ]] || die "ROS Noetic setup file was not found"

source /opt/ros/noetic/setup.bash
source "$SIMENV/devel/setup.bash"
source "$WSLIVOX/devel/setup.bash"
# ws_livox's setup.bash overwrote SimEnv's CMAKE_PREFIX_PATH entry, so roslaunch
# can no longer locate SimEnv's catkin-installed relay scripts
# (devel/lib/<pkg>/<node>.py) via catkin_find. Restore both devel spaces in
# front so roslaunch/rospack resolve node executables from both workspaces.
export CMAKE_PREFIX_PATH="$SIMENV/devel:$WSLIVOX/devel:$CMAKE_PREFIX_PATH"
# Merge the two independent workspaces' src paths after the setup scripts
# have overwritten each other's ROS_PACKAGE_PATH.
export ROS_PACKAGE_PATH="$SIMENV/src:$WSLIVOX/src:$ROS_PACKAGE_PATH"

command -v roslaunch >/dev/null 2>&1 || die "roslaunch is unavailable after sourcing ROS Noetic"
rospack find fastlio2_merge >/dev/null 2>&1 || \
  die "ROS package fastlio2_merge is unavailable; install and build explorer-v1 first"
rospack find fast_lio2 >/dev/null 2>&1 || \
  die "ROS package fast_lio2 is unavailable; build ws_livox first"

echo "Starting FAST-LIO2 SLAM + competition navigation"
echo "  SimEnv: $SIMENV"
echo "  ws_livox: $WSLIVOX"
exec roslaunch fastlio2_merge slam_nav.launch "${EXTRA_ARGS[@]}"
