# competition_navigation_hazard_v0

整合启动包：FAST-LIO2 SLAM + C++ 导航 + **v0 危险源监测**。

这是 `competition_navigation_hazard` 的 v0-hazard 变体。唯一区别是危险源检测
由 **`hazard_perception_v0`** 提供（从 `~/explorer-v0` 复制而来，核心检测算法
与当前 `hazard_perception` 一致，额外增加了 completion / tf-bridge / finalize-cli
等集成工具）。导航逻辑、完成触发逻辑与原来完全一致。

## 启动

仿真环境运行后（且 ws_livox 已 source）：

```bash
source /opt/ros/noetic/setup.bash
source $HOME/SimEnv/devel/setup.bash
source $HOME/ws_livox/devel/setup.bash
roslaunch competition_navigation_hazard_v0 slam_nav_hazard_v0.launch
```

也可以使用现有脚本风格直接 launch。

## 组成

- `launch/slam_nav_hazard_v0.launch` — 整合入口：
  1. `competition_navigation_fastlio2_cpp/slam_nav.launch`（SLAM + C++ 导航 + save_pcd）
  2. `hazard_perception_v0/hazard_pipeline.launch`（红球 RGB-D 检测 + 定位跟踪）
  3. `hazard_finalize_trigger.py`（探索完成 → 自动调用 `/hazard_perception_v0/finalize`）
- `scripts/hazard_finalize_trigger.py` — 监听 `/competition_navigation/status`，
  检测到 `full_map_complete` / `returned_home` 后触发 finalize。

## 结果

危险源结果默认写入 `~/SimEnv/results/hazard_v0/`（launch 参数 `output_directory`
可改）。完成后可用真值文件与检测结果比较。

## 相关包

- `hazard_perception_v0`：v0 危险源检测（本包依赖）
- `hazard_perception`：当前（v1）危险源检测，未被修改
- `competition_navigation_fastlio2_cpp`：C++ 导航
