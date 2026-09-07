#!/usr/bin/env bash
# 一键：先清掉上一轮仿真残留，再启动本仿真(带 GUI + 站立/RL)。
# 用法：bash /home/loser/SimEnv/scripts/start_sim_clean.sh
set -u
bash "$(dirname "$0")/cleanup_sim.sh"
echo "== starting sim =="
exec python3 /tmp/start_sim.py
