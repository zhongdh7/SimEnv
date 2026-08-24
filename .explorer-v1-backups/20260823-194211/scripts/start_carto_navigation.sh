#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Start loop-closure-corrected navigation (FAST-LIO2 + Cartographer + nav).

Runs FAST-LIO2 as the odometry source, Cartographer for loop-closure-corrected
localisation (TF carto_map -> base), carto_relay to republish that pose as
/carto_odom and the scan as /scan_carto, and the competition navigation node
localised on carto_map.  The
exploration logic is the same competition_navigation_node.py, re-wired to the
carto_map frame.

FAST-LIO2 lives in fast_lio2 (ws_livox), Cartographer in ~/cartographer_ws
(install_isolated), and everything else in SimEnv.  Sourcing order matters:
these are independent catkin workspaces whose setup.bash files overwrite each
other's ROS_PACKAGE_PATH, so this script sources all three and merges their
paths explicitly.

Usage:
  start_carto_navigation.sh [--simenv PATH] [--wslivox PATH] [--carto PATH] [-- roslaunch-arg:=value ...]

Options:
  --simenv PATH     SimEnv workspace (default: $HOME/SimEnv)
  --wslivox PATH    ws_livox workspace containing FAST-LIO2 (default: $HOME/ws_livox)
  --carto PATH      Cartographer isolated install (default: $HOME/cartographer_ws/install_isolated)
  -h, --help        Show this help

The Gazebo competition environment and junior_ctrl must already be running.
Use -- to pass additional arguments to carto_nav.launch (e.g. rviz:=true).
EOF
}

die() {
  echo "start_carto_navigation: $*" >&2
  exit 2
}

SIMENV="${SIMENV:-$HOME/SimEnv}"
WSLIVOX="${WSLIVOX:-$HOME/ws_livox}"
CARTO="${CARTO:-$HOME/cartographer_ws/install_isolated}"
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
    --carto)
      [[ $# -ge 2 ]] || die "--carto requires a path"
      CARTO="$2"
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
[[ -f "$CARTO/setup.bash" ]] || die "missing $CARTO/setup.bash; install Cartographer first"
[[ -f /opt/ros/noetic/setup.bash ]] || die "ROS Noetic setup file was not found"

source /opt/ros/noetic/setup.bash
source "$SIMENV/devel/setup.bash"
source "$WSLIVOX/devel/setup.bash"
source "$CARTO/setup.bash"
# Each independent workspace's setup.bash overwrote the previous one's entries;
# restore all devel/install spaces in front so roslaunch/rospack resolve node
# executables from all three workspaces.
export CMAKE_PREFIX_PATH="$SIMENV/devel:$WSLIVOX/devel:$CARTO:$CMAKE_PREFIX_PATH"
# Merge the workspaces' src paths (SimEnv carries carto_localization and the nav
# packages; ws_livox carries fast_lio2) ahead of the overwritten paths.
export ROS_PACKAGE_PATH="$SIMENV/src:$WSLIVOX/src:$ROS_PACKAGE_PATH"

command -v roslaunch >/dev/null 2>&1 || die "roslaunch is unavailable after sourcing ROS Noetic"
rospack find carto_localization >/dev/null 2>&1 || \
  die "ROS package carto_localization is unavailable; install and build explorer-v1 first"
rospack find competition_navigation_fastlio2 >/dev/null 2>&1 || \
  die "ROS package competition_navigation_fastlio2 is unavailable; install and build explorer-v1 first"
rospack find fast_lio2 >/dev/null 2>&1 || \
  die "ROS package fast_lio2 is unavailable; build ws_livox first"
rospack find cartographer_ros >/dev/null 2>&1 || \
  die "ROS package cartographer_ros is unavailable; check --carto path"

echo "Starting loop-closure-corrected navigation (FAST-LIO2 + Cartographer + nav)"
echo "  SimEnv:       $SIMENV"
echo "  ws_livox:     $WSLIVOX"
echo "  Cartographer: $CARTO"
exec roslaunch carto_localization carto_nav.launch "${EXTRA_ARGS[@]}"
