#!/usr/bin/env bash
set -euo pipefail

# 整合启动：FAST-LIO2 SLAM + C++ 导航 + 危险源监测（浅整合）。
#
# 路径规则（优先级从高到低）：
#   1. --simenv / --wslivox / --hdplanner-root 命令行参数；
#   2. SIMENV / WSLIVOX / HDPLANNER_ROOT 环境变量；
#   3. 安装时写入本目录的 explorer_paths.env（脚本被 install_into_simenv.sh
#      复制到 <SimEnv>/scripts/ 后由 installer 生成）；
#   4. 由脚本自身位置推导（<SimEnv>/scripts/ 的父目录）；
#   5. $HOME/SimEnv 等 convenience 默认值。
#
# 关键点：ws_livox 的 devel/setup.bash 会覆盖 SimEnv 的 PYTHONPATH，导致
# hazard_perception 这个 Python 包找不到 (ModuleNotFoundError)。因此这里在
# source 完两个 workspace 之后，显式把 SimEnv 的 Python 包路径放回最前面。

usage() {
  cat <<'EOF'
Usage: start_slam_navigation_hazard.sh [options] [--] [roslaunch args...]

Options:
  --simenv PATH         SimEnv workspace
  --wslivox PATH        ws_livox workspace (FAST-LIO2)
  --hdplanner-root PATH HDPlanner model root
  --print-paths         Print resolved paths and exit without launching
  -h, --help            Show this help and exit

Examples:
  start_slam_navigation_hazard.sh --simenv /data/SimEnv \
    --wslivox /opt/ws_livox --hdplanner-root /data/HDPlanner_Exp_and_Nav \
    -- auto_start:=true start_verifier:=false rviz:=false

  SIMENV=/data/SimEnv WSLIVOX=/opt/ws_livox HDPLANNER_ROOT=/data/HDPlanner \
    start_slam_navigation_hazard.sh auto_start:=true rviz:=false
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALLED_PREFIX=""
if [[ -f "$SCRIPT_DIR/../auto.sh" && -d "$SCRIPT_DIR/../src" ]]; then
  INSTALLED_PREFIX="$(cd "$SCRIPT_DIR/.." && pwd)"
fi

SIMENV_ARG=""
WSLIVOX_ARG=""
HDPLANNER_ROOT_ARG=""
PRINT_PATHS=0
LAUNCH_ARGS=()
END_OF_OPTIONS=0
while [[ $# -gt 0 ]]; do
  if [[ $END_OF_OPTIONS -eq 1 ]]; then
    LAUNCH_ARGS+=("$1")
    shift
    continue
  fi
  case "$1" in
    --simenv)
      [[ $# -ge 2 ]] || { echo "--simenv requires a path" >&2; exit 2; }
      SIMENV_ARG="$2"
      shift 2
      ;;
    --wslivox)
      [[ $# -ge 2 ]] || { echo "--wslivox requires a path" >&2; exit 2; }
      WSLIVOX_ARG="$2"
      shift 2
      ;;
    --hdplanner-root)
      [[ $# -ge 2 ]] || { echo "--hdplanner-root requires a path" >&2; exit 2; }
      HDPLANNER_ROOT_ARG="$2"
      shift 2
      ;;
    --print-paths)
      PRINT_PATHS=1
      shift
      ;;
    --)
      END_OF_OPTIONS=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      # Backward-compatible positional roslaunch args such as
      # auto_start:=true (no leading dash) are passed through.
      LAUNCH_ARGS+=("$1")
      shift
      ;;
  esac
done

# Load install-time path metadata only as a fallback.  CLI args and the
# caller's exported environment take precedence below.
ENV_SIMENV="${SIMENV:-}"
ENV_WSLIVOX="${WSLIVOX:-}"
ENV_HDPLANNER_ROOT="${HDPLANNER_ROOT:-}"
CONFIG_FILE="$SCRIPT_DIR/explorer_paths.env"
if [[ -f "$CONFIG_FILE" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "$CONFIG_FILE"
  set +a
fi
if [[ -n "$ENV_SIMENV" ]]; then SIMENV="$ENV_SIMENV"; fi
if [[ -n "$ENV_WSLIVOX" ]]; then WSLIVOX="$ENV_WSLIVOX"; fi
if [[ -n "$ENV_HDPLANNER_ROOT" ]]; then HDPLANNER_ROOT="$ENV_HDPLANNER_ROOT"; fi

if [[ -n "$SIMENV_ARG" ]]; then
  SIMENV="$SIMENV_ARG"
elif [[ -z "${SIMENV:-}" ]]; then
  if [[ -n "$INSTALLED_PREFIX" ]]; then
    SIMENV="$INSTALLED_PREFIX"
  else
    SIMENV="$HOME/SimEnv"
  fi
fi

if [[ -n "$WSLIVOX_ARG" ]]; then
  WSLIVOX="$WSLIVOX_ARG"
elif [[ -z "${WSLIVOX:-}" ]]; then
  WSLIVOX="$HOME/ws_livox"
fi

if [[ -n "$HDPLANNER_ROOT_ARG" ]]; then
  HDPLANNER_ROOT="$HDPLANNER_ROOT_ARG"
elif [[ -z "${HDPLANNER_ROOT:-}" ]]; then
  HDPLANNER_ROOT="$HOME/HDPlanner_Exp_and_Nav"
fi

if [[ "$PRINT_PATHS" == "1" ]]; then
  printf 'SIMENV=%s\n' "$SIMENV"
  printf 'WSLIVOX=%s\n' "$WSLIVOX"
  printf 'HDPLANNER_ROOT=%s\n' "$HDPLANNER_ROOT"
  exit 0
fi

[[ -f "$SIMENV/devel/setup.bash" ]] || { echo "missing $SIMENV/devel/setup.bash" >&2; exit 2; }
[[ -f "$WSLIVOX/devel/setup.bash" ]] || { echo "missing $WSLIVOX/devel/setup.bash" >&2; exit 2; }

export SIMENV
export WSLIVOX
export HDPLANNER_ROOT

set +u
source /opt/ros/noetic/setup.bash
source "$SIMENV/devel/setup.bash"
source "$WSLIVOX/devel/setup.bash"
set -u

# 恢复两个 workspace 的包搜索路径（ws_livox 覆盖了 SimEnv 的条目）。
export CMAKE_PREFIX_PATH="$SIMENV/devel:$WSLIVOX/devel:$CMAKE_PREFIX_PATH"
export ROS_PACKAGE_PATH="$SIMENV/src:$WSLIVOX/src:$ROS_PACKAGE_PATH"
# 恢复 SimEnv 的 Python 包路径，否则 hazard_perception 无法 import。
export PYTHONPATH="$SIMENV/devel/lib/python3/dist-packages:$PYTHONPATH"

exec roslaunch competition_navigation_hazard slam_nav_hazard.launch "${LAUNCH_ARGS[@]}"
