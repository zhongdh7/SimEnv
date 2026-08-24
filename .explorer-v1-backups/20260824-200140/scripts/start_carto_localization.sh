#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Start loop-closure-corrected 2-D localisation (Cartographer) for SimEnv.

Runs pointcloud_to_laserscan.py + cartographer_node +
cartographer_occupancy_grid_node to produce a drift-free carto_map frame via
loop closure.  Independent of FAST-LIO and of the navigation stack; does not
touch competition_navigation_node.py.

Prerequisites:
  - the SimEnv Gazebo environment is already running (/scan + base->laser_livox TF)
  - Cartographer is installed at ~/cartographer_ws (sourced from install_isolated)

Usage:
  start_carto_localization.sh [--simenv PATH] [--carto PATH] [-- roslaunch-arg:=value ...]

Options:
  --simenv PATH   SimEnv workspace (default: $HOME/SimEnv)
  --carto PATH    Cartographer isolated install (default: $HOME/cartographer_ws/install_isolated)
  -h, --help      Show this help

Use -- to pass additional arguments to carto_locate.launch (e.g. rviz:=true).
EOF
}

die() {
  echo "start_carto_localization: $*" >&2
  exit 2
}

SIMENV="${SIMENV:-$HOME/SimEnv}"
CARTO="${CARTO:-$HOME/cartographer_ws/install_isolated}"
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --simenv)
      [[ $# -ge 2 ]] || die "--simenv requires a path"
      SIMENV="$2"
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
[[ -f "$CARTO/setup.bash" ]] || die "missing $CARTO/setup.bash; install Cartographer first"
[[ -f /opt/ros/noetic/setup.bash ]] || die "ROS Noetic setup file was not found"

source /opt/ros/noetic/setup.bash
source "$SIMENV/devel/setup.bash"
source "$CARTO/setup.bash"
# Restore SimEnv's src ahead of Cartographer's share dirs so roslaunch resolves
# carto_localization from SimEnv while cartographer_ros resolves from CARTO.
export ROS_PACKAGE_PATH="$SIMENV/src:$ROS_PACKAGE_PATH"

command -v roslaunch >/dev/null 2>&1 || die "roslaunch is unavailable after sourcing ROS Noetic"
rospack find carto_localization >/dev/null 2>&1 || \
  die "ROS package carto_localization is unavailable; build it in SimEnv first"
rospack find cartographer_ros >/dev/null 2>&1 || \
  die "ROS package cartographer_ros is unavailable; check --carto path"

echo "Starting Cartographer loop-closure localisation"
echo "  SimEnv:       $SIMENV"
echo "  Cartographer: $CARTO"
exec roslaunch carto_localization carto_locate.launch "${EXTRA_ARGS[@]}"
