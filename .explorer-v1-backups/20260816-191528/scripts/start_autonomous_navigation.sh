#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Start the competition autonomous navigation stack.

Usage:
  start_autonomous_navigation.sh [options] [-- roslaunch-arg:=value ...]

Options:
  --simenv PATH          SimEnv workspace (default: $HOME/SimEnv)
  --manual-start         Start the node without beginning exploration
  --no-verifier          Do not start the full-map acceptance verifier
  --model-path PATH      HDPlanner TorchScript checkpoint
  --library-path PATH    HDPlanner inference shared library
  --layout-path PATH     Generated building layout metadata
  --report-path PATH     Full-map report output path
  --wait-seconds N       Preflight wait limit for ROS and sensor topics (30)
  --skip-preflight       Skip ROS master and sensor topic checks
  -h, --help             Show this help

The Gazebo competition environment and junior_ctrl must already be running.
Use -- to pass additional arguments to competition_navigation.launch.
EOF
}

die() {
  echo "start_autonomous_navigation: $*" >&2
  exit 2
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
if [[ -f "$SCRIPT_ROOT/auto.sh" ]]; then
  DEFAULT_SIMENV="$SCRIPT_ROOT"
else
  DEFAULT_SIMENV="${HOME}/SimEnv"
fi

SIMENV="${SIMENV:-$DEFAULT_SIMENV}"
MODEL_PATH_OVERRIDE=""
LIBRARY_PATH_OVERRIDE=""
LAYOUT_PATH_OVERRIDE=""
REPORT_PATH_OVERRIDE=""
AUTO_START=true
START_VERIFIER=true
WAIT_SECONDS=30
SKIP_PREFLIGHT=false
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --simenv)
      [[ $# -ge 2 ]] || die "--simenv requires a path"
      SIMENV="$2"
      shift 2
      ;;
    --manual-start)
      AUTO_START=false
      shift
      ;;
    --no-verifier)
      START_VERIFIER=false
      shift
      ;;
    --model-path)
      [[ $# -ge 2 ]] || die "--model-path requires a path"
      MODEL_PATH_OVERRIDE="$2"
      shift 2
      ;;
    --library-path)
      [[ $# -ge 2 ]] || die "--library-path requires a path"
      LIBRARY_PATH_OVERRIDE="$2"
      shift 2
      ;;
    --layout-path)
      [[ $# -ge 2 ]] || die "--layout-path requires a path"
      LAYOUT_PATH_OVERRIDE="$2"
      shift 2
      ;;
    --report-path)
      [[ $# -ge 2 ]] || die "--report-path requires a path"
      REPORT_PATH_OVERRIDE="$2"
      shift 2
      ;;
    --wait-seconds)
      [[ $# -ge 2 ]] || die "--wait-seconds requires a number"
      WAIT_SECONDS="$2"
      shift 2
      ;;
    --skip-preflight)
      SKIP_PREFLIGHT=true
      shift
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

[[ "$WAIT_SECONDS" =~ ^[0-9]+$ ]] || die "--wait-seconds must be a non-negative integer"
[[ -d "$SIMENV" ]] || die "SimEnv workspace does not exist: $SIMENV"
[[ -f "$SIMENV/devel/setup.bash" ]] || die "missing $SIMENV/devel/setup.bash; build SimEnv first"
[[ -f /opt/ros/noetic/setup.bash ]] || die "ROS Noetic setup file was not found"

MODEL_PATH="${MODEL_PATH_OVERRIDE:-${HDPLANNER_MODEL_PATH:-$HOME/HDPlanner_Exp_and_Nav/model/HDPlanner_Nav/policy_traced.pt}}"
LIBRARY_PATH="${LIBRARY_PATH_OVERRIDE:-${HDPLANNER_LIBRARY_PATH:-$SIMENV/devel/lib/libhdplanner_inference.so}}"
LAYOUT_PATH="${LAYOUT_PATH_OVERRIDE:-${COMPETITION_LAYOUT_PATH:-$SIMENV/generated_building/layout_metadata.json}}"
REPORT_PATH="${REPORT_PATH_OVERRIDE:-${COMPETITION_REPORT_PATH:-$SIMENV/results/full_map_coverage.json}}"

source /opt/ros/noetic/setup.bash
source "$SIMENV/devel/setup.bash"

command -v roslaunch >/dev/null 2>&1 || die "roslaunch is unavailable after sourcing ROS Noetic"
rospack find competition_navigation >/dev/null 2>&1 || \
  die "ROS package competition_navigation is unavailable; install and build explorer-v1 first"
[[ -f "$MODEL_PATH" ]] || die "HDPlanner model was not found: $MODEL_PATH"
[[ -f "$LIBRARY_PATH" ]] || die "HDPlanner inference library was not found: $LIBRARY_PATH"
mkdir -p "$(dirname -- "$REPORT_PATH")"

wait_for_master() {
  local attempt
  for ((attempt = 0; attempt <= WAIT_SECONDS; attempt++)); do
    if rosparam get /rosversion >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_topic() {
  local topic="$1"
  local attempt
  for ((attempt = 0; attempt <= WAIT_SECONDS; attempt++)); do
    if rostopic list 2>/dev/null | grep -Fxq "$topic"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

if [[ "$SKIP_PREFLIGHT" == false ]]; then
  echo "Waiting for ROS master..."
  wait_for_master || die "ROS master is unavailable; start the competition environment with ./auto.sh"
  echo "Waiting for /scan and /Odometry_gazebo..."
  wait_for_topic /scan || die "sensor topic /scan was not found"
  wait_for_topic /Odometry_gazebo || die "odometry topic /Odometry_gazebo was not found"
  [[ -f "$LAYOUT_PATH" ]] || die "generated layout metadata was not found: $LAYOUT_PATH"
fi

if rosservice call /gazebo/unpause_physics >/dev/null 2>&1; then
  echo "Gazebo physics unpaused"
else
  die "Gazebo physics could not be unpaused"
fi

LAUNCH_ARGS=(
  "auto_start:=$AUTO_START"
  "start_verifier:=$START_VERIFIER"
  "require_hdplanner:=true"
  "model_path:=$MODEL_PATH"
  "library_path:=$LIBRARY_PATH"
  "layout_metadata_path:=$LAYOUT_PATH"
  "coverage_report_path:=$REPORT_PATH"
)

echo "Starting competition_navigation"
echo "  SimEnv: $SIMENV"
echo "  auto_start: $AUTO_START"
echo "  verifier: $START_VERIFIER"
echo "  report: $REPORT_PATH"
exec roslaunch competition_navigation competition_navigation.launch "${LAUNCH_ARGS[@]}" "${EXTRA_ARGS[@]}"
