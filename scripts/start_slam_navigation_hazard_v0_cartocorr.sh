#!/usr/bin/env bash
set -euo pipefail

# 整合启动：FAST-LIO2 SLAM + Cartographer 回环修正 + C++ 导航 + v0 危险源监测。
#
# 与 start_slam_navigation_hazard_v0.sh 的区别：额外启动 Cartographer 回环定位，
# carto_correction 把"Cartographer 认为 FAST-LIO2 漂移的 x/y"算成平滑修正量，
# scan_tf_relay 叠加进 /Odometry_fastlio —— 导航实时位姿仍是 FAST-LIO2，只是被
# 回环低频纠偏（不会像 carto_nav 那样整体切到 2D Cartographer 位姿而撞墙）。

SIMENV="${SIMENV:-$HOME/SimEnv}"
WSLIVOX="${WSLIVOX:-$HOME/ws_livox}"
CARTO="${CARTO:-$HOME/cartographer_ws/install_isolated}"

[[ -f "$SIMENV/devel/setup.bash" ]] || { echo "missing $SIMENV/devel/setup.bash" >&2; exit 2; }
[[ -f "$WSLIVOX/devel/setup.bash" ]] || { echo "missing $WSLIVOX/devel/setup.bash" >&2; exit 2; }
[[ -f "$CARTO/setup.bash" ]] || { echo "missing $CARTO/setup.bash (install Cartographer first)" >&2; exit 2; }

source /opt/ros/noetic/setup.bash
source "$SIMENV/devel/setup.bash"
source "$WSLIVOX/devel/setup.bash"
source "$CARTO/setup.bash"

# 三个独立工作区 setup.bash 互相覆盖 ROS_PACKAGE_PATH / CMAKE_PREFIX_PATH，
# 显式把三者的 devel/install 放回最前面，并恢复 SimEnv 的 Python 包路径。
export CMAKE_PREFIX_PATH="$SIMENV/devel:$WSLIVOX/devel:$CARTO:$CMAKE_PREFIX_PATH"
export ROS_PACKAGE_PATH="$SIMENV/src:$WSLIVOX/src:$ROS_PACKAGE_PATH"
export PYTHONPATH="$SIMENV/devel/lib/python3/dist-packages:$PYTHONPATH"

exec roslaunch competition_navigation_hazard_v0 \
  slam_nav_hazard_v0_cartocorr.launch "$@"
