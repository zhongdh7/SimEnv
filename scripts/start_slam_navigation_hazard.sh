#!/usr/bin/env bash
set -euo pipefail

# 整合启动：FAST-LIO2 SLAM + C++ 导航 + 危险源监测（浅整合）。
#
# 导航 (competition_navigation_fastlio2_cpp) 与危险源检测 (hazard_perception)
# 各自独立运行，本脚本只是把它们一起拉起。
#
# 关键点：ws_livox 的 devel/setup.bash 会覆盖 SimEnv 的 PYTHONPATH，导致
# hazard_perception 这个 Python 包找不到 (ModuleNotFoundError)。因此这里在
# source 完两个 workspace 之后，显式把 SimEnv 的 Python 包路径放回最前面。

SIMENV="${SIMENV:-$HOME/SimEnv}"
WSLIVOX="${WSLIVOX:-$HOME/ws_livox}"

[[ -f "$SIMENV/devel/setup.bash" ]] || { echo "missing $SIMENV/devel/setup.bash" >&2; exit 2; }
[[ -f "$WSLIVOX/devel/setup.bash" ]] || { echo "missing $WSLIVOX/devel/setup.bash" >&2; exit 2; }

source /opt/ros/noetic/setup.bash
source "$SIMENV/devel/setup.bash"
source "$WSLIVOX/devel/setup.bash"

# 恢复两个 workspace 的包搜索路径（ws_livox 覆盖了 SimEnv 的条目）。
export CMAKE_PREFIX_PATH="$SIMENV/devel:$WSLIVOX/devel:$CMAKE_PREFIX_PATH"
export ROS_PACKAGE_PATH="$SIMENV/src:$WSLIVOX/src:$ROS_PACKAGE_PATH"
# 恢复 SimEnv 的 Python 包路径，否则 hazard_perception 无法 import。
export PYTHONPATH="$SIMENV/devel/lib/python3/dist-packages:$PYTHONPATH"

exec roslaunch competition_navigation_hazard slam_nav_hazard.launch "$@"
