# competition_navigation_hazard

整合启动包：在导航探索的同时运行危险源监测，验证探索过程中能否检测到危险源（红球）。

## 作用

用一个 `roslaunch` 同时拉起：

1. **导航**（`competition_navigation_fastlio2_cpp`）——FAST-LIO2 SLAM + C++ 导航节点，负责三层楼探索与返航。
2. **危险源监测**（`hazard_perception`）——RGB-D 红球检测 + 定位跟踪 + 结果管理。

两者**独立运行、互不改代码**（浅整合）。本包只提供一个统一的启动入口，不含任何节点逻辑。

## 启动

前置：SimEnv 仿真 + `junior_ctrl` 已运行，且已 source `ws_livox`。

```bash
source /opt/ros/noetic/setup.bash
source $HOME/SimEnv/devel/setup.bash
source $HOME/ws_livox/devel/setup.bash
roslaunch competition_navigation_hazard slam_nav_hazard.launch
```

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `rviz` | `false` | 是否打开导航的 RViz |
| `auto_start` | `true` | 导航是否自动开始 |
| `start_verifier` | `true` | 是否启动验证节点 |
| `use_livox` | `false` | 危险源检测是否融合 Livox 点云 |
| `output_directory` | `~/SimEnv/results/hazard` | 危险源结果输出目录 |

## 验证危险源检测

导航运行期间，检查危险源检测话题：

```bash
rostopic echo -n1 /hazard_perception/detections   # 2D 检测结果
rostopic echo -n1 /hazard_perception/hazards_3d   # 3D 定位结果
```

## 依赖

- `competition_navigation_fastlio2_cpp`（C++ 导航）
- `hazard_perception`（危险源检测）
- `fastlio2_merge`（FAST-LIO2 仿真 SLAM 与 save_pcd）
