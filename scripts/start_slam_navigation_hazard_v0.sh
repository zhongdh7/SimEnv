#!/usr/bin/env bash
set -euo pipefail

# 整合启动：FAST-LIO2 SLAM + C++ 导航 + v0 危险源监测（浅整合）。
#
# 与 start_slam_navigation_hazard.sh 的唯一区别：危险源检测使用
# hazard_perception_v0（从 explorer-v0 复制），finalize 服务为
# /hazard_perception_v0/finalize。导航逻辑与完成触发逻辑完全一致。

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
# 恢复 SimEnv 的 Python 包路径，否则 hazard_perception_v0 无法 import。
export PYTHONPATH="$SIMENV/devel/lib/python3/dist-packages:$PYTHONPATH"

exec roslaunch competition_navigation_hazard_v0 slam_nav_hazard_v0.launch "$@"
