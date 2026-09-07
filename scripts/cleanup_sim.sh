#!/usr/bin/env bash
# 启动仿真/导航前清理残留的仿真与 ROS 进程 + 日志，确保每次从干净环境起。
# 用法：bash /home/loser/SimEnv/scripts/cleanup_sim.sh
# 也会被 start_sim.py 自动调用（每次起 sim 前先清一次）。
set -u
SELF=$$
# 收集"本次调用链"：不杀自己/父进程链（如正在调用本脚本的 start_sim.py 及其 shell）。
SKIP=" $SELF "
CPID=$$
for _ in 1 2 3 4 5 6 7 8; do
  PPID_N=$(ps -o ppid= -p "$CPID" 2>/dev/null | tr -d ' ')
  [ -z "$PPID_N" ] || [ "$PPID_N" = "0" ] || [ "$PPID_N" = "1" ] && break
  SKIP="$SKIP$PPID_N "
  CPID=$PPID_N
done
echo "== cleanup_sim.sh: killing leftover sim/ros processes (skip=$SKIP) =="

# 第一轮：尽可能杀掉常见仿真/导航进程（排除自身）
for pat in \
  "rosmaster" "roslaunch" "gzserver" "gzclient" "junior_ctrl" "start_sim.py" \
  "state_from_gazebo" "controller_spawner" "robot_state_publisher" \
  "pointcloud2livox" "pointcloud_converter" "imu_gravity_compensator" \
  "fastlio2_mapping" "frame_republisher" "scan_tf_relay" \
  "competition_navigation_node" "collision_safety" "full_map_verifier" \
  "save_pcd" "hazard_pipeline_node" "hazard_finalize_trigger" \
  "monitor_fall" "auto.sh" "multi_floor_gazeboSim" "carto_localization" \
  "carto_relay" "carto_correction" ; do
  for p in $(pgrep -f "$pat" 2>/dev/null); do
    case " $SKIP " in *" $p "*) ;; *) kill -9 "$p" 2>/dev/null ;; esac
  done
done
sleep 4

# 第二轮：仍活着的核心仿真进程（可能被上轮杀掉后重新拉起的子进程）
for pat in "rosmaster" "gzserver" "junior_ctrl" "start_sim.py"; do
  for p in $(pgrep -f "$pat" 2>/dev/null); do
    case " $SKIP " in *" $p "*) ;; *) kill -9 "$p" 2>/dev/null ;; esac
  done
done
sleep 2

# 清掉 ~/.ros 里的会话日志，避免几十万残留文件
if [ -d "$HOME/.ros/log" ]; then
  rm -rf "$HOME/.ros/log" 2>/dev/null
  mkdir -p "$HOME/.ros/log"
fi

echo "== cleanup_sim.sh: done (env should be clean) =="
pgrep -af "rosmaster|gzserver|roslaunch|junior_ctrl" 2>/dev/null | grep -v "cleanup_sim.sh" | head -3 || true
