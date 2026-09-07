#!/usr/bin/env bash
set -euo pipefail

# 启动：相机覆盖率导航（competition_navigation_fastlio2_cam）+ 危险源监测。
#
# 与 start_slam_navigation_fastlio2_cpp.sh 相同，但导航节点换成了相机覆盖率
# 变体（房间探索完成判定改用 RGB 相机 FOV 覆盖，而非 LiDAR 覆盖率）。
#
# 关键点同 cpp 版：ws_livox 的 setup.bash 会覆盖 SimEnv 的 PYTHONPATH，导致
# hazard_perception 无法 import，因此显式把 SimEnv 的 Python 包路径放回最前面。

SIMENV="${SIMENV:-$HOME/SimEnv}"
WSLIVOX="${WSLIVOX:-$HOME/ws_livox}"

[[ -f "$SIMENV/devel/setup.bash" ]] || { echo "missing $SIMENV/devel/setup.bash" >&2; exit 2; }
[[ -f "$WSLIVOX/devel/setup.bash" ]] || { echo "missing $WSLIVOX/devel/setup.bash" >&2; exit 2; }

source /opt/ros/noetic/setup.bash
source "$SIMENV/devel/setup.bash"
source "$WSLIVOX/devel/setup.bash"

export CMAKE_PREFIX_PATH="$SIMENV/devel:$WSLIVOX/devel:$CMAKE_PREFIX_PATH"
export ROS_PACKAGE_PATH="$SIMENV/src:$WSLIVOX/src:$ROS_PACKAGE_PATH"
export PYTHONPATH="$SIMENV/devel/lib/python3/dist-packages:$PYTHONPATH"

exec roslaunch competition_navigation_fastlio2_cam slam_nav_hazard.launch "$@"
