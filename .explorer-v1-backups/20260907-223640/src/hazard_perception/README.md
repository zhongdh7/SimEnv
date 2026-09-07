# hazard_perception

ROS Noetic 红色球形危险源二维检测、RGB-D/Livox 三维定位、多帧跟踪和结果保存包。

完整工作区说明见仓库根目录 [README](../../README.md)，算法细节见
[检测方法与原理](../../docs/DETECTION_METHOD.md)，接入其他节点见
[团队集成指南](../../docs/TEAM_INTEGRATION.md)。

## 节点

### `hazard_pipeline_node.py`（正式入口）

同步 RGB、Depth 和两路 CameraInfo，执行：

```text
HSV红色候选 -> 二维形状/触边圆弧 -> RGB-D球面几何验证
-> 可选Livox融合 -> TF到start_frame -> 多帧确认/去重 -> 最终结果
```

正式节点不订阅 `/gazebo/model_states`，不读取危险源真值，也不控制机器人运动。

### `hazard_detector_node.py`（仅二维调试）

只订阅 RGB，发布红色 mask 和二维候选。它不产生三维位置，也不做球面深度验证。
不要和完整流水线同时以默认话题启动。

## 编译

```bash
export SIMENV_ROOT=/absolute/path/to/SimEnv
export HAZARD_WS=/absolute/path/to/hazard_detection_v1

cd "$HAZARD_WS"
source /opt/ros/noetic/setup.bash
source "$SIMENV_ROOT/devel/setup.bash"
catkin_make --pkg hazard_perception -j2
source devel/setup.bash
```

## 启动

纯 RGB-D：

```bash
roslaunch hazard_perception hazard_pipeline.launch \
  use_livox:=false \
  publish_debug_images:=true \
  publish_candidates:=true \
  rviz:=true
```

RGB-D + Livox（当前 SimEnv `/scan` 为 `sensor_msgs/PointCloud` 时）：

```bash
roslaunch hazard_perception hazard_pipeline.launch \
  use_livox:=true \
  livox_topic:=/scan \
  livox_message_type:=pointcloud \
  rviz:=true
```

二维调试：

```bash
roslaunch hazard_perception hazard_detector.launch
```

## 输入

| 默认话题 | 类型 |
| --- | --- |
| `/real_sense/rgb/image_raw` | `sensor_msgs/Image` |
| `/real_sense/depth/image_raw` | `sensor_msgs/Image` |
| `/real_sense/rgb/camera_info` | `sensor_msgs/CameraInfo` |
| `/real_sense/depth/camera_info` | `sensor_msgs/CameraInfo` |
| `/real_sense/depth/points` | `sensor_msgs/PointCloud2`，非对齐回退 |
| `/scan` | `PointCloud`/`PointCloud2`，可选 |
| `/tf`, `/tf_static` | 必须连通 odom、base、相机和可选 Livox |

## 输出

| 默认名称 | 类型 | 含义 |
| --- | --- | --- |
| `/hazard_perception/detections` | `HazardDetection2DArray` | 当前帧二维候选 |
| `/hazard_perception/hazards_3d` | `Hazard3DArray` | 跟踪后的三维目标 |
| `/hazard_perception/geometry_diagnostics` | `std_msgs/String` | JSON 几何判定依据 |
| `/hazard_perception/markers` | `MarkerArray` | RViz 标记 |
| `/hazard_perception/pipeline_red_mask` | `sensor_msgs/Image` | 完整流水线 mask |
| `/hazard_perception/pipeline_debug_image` | `sensor_msgs/Image` | 完整流水线调试图 |
| `/hazard_perception/finalize` | `std_srvs/Trigger` | 最终去重并保存 |

`hazards_3d` 在 `publish_candidates=true` 时会包含未确认候选。正式消费者必须检查
`confirmed`，不能只判断数组非空。

## 配置

- `config/red_sphere.yaml`：HSV、形态学、轮廓、深度和球面验证。
- `config/localization_tracking.yaml`：坐标系、确认、平滑、关联和去重。
- `config/fusion.yaml`：Livox 聚类、球面验证和融合。

当前默认比赛球半径为 `0.15 m`，RGB-D 使用已知半径稳健拟合。明确不符合球面结构的候选不会进入
tracker；深度不足的回退候选不会仅靠重复帧成为 confirmed。

## 保存结果

```bash
rosservice call /hazard_perception/finalize '{}'
```

launch 参数 `output_directory`、`run_id` 和 `save_csv` 控制输出。内部 JSON 格式不保证与赛方提交格式
相同，应由 integration 层适配。

## 测试

```bash
cd "$HAZARD_WS"
source /opt/ros/noetic/setup.bash
source "$SIMENV_ROOT/devel/setup.bash"
source devel/setup.bash

python3 -m unittest discover -s src/hazard_perception/test -p 'test_*.py' -v
catkin_make run_tests_hazard_perception
catkin_test_results build/test_results/hazard_perception
```

上述单元测试不读取 Gazebo 真值。在线黑盒验证方法见根 README；
`test_tools/hazard_validation.py` 会读取真值并操作 Gazebo 模型，只能用于规则明确允许的隔离开发评测，
不得进入正式比赛、共享联调或真值无关验证。

## 关键提醒

- 机器人复位后必须重启 pipeline，重新捕获 `start_frame`。
- 二维 debug 图上的框不等于最终 confirmed 三维目标。
- 深度编码只支持 `32FC1` 米或 `16UC1` 毫米。
- RGB/Depth/CameraInfo 时间差默认不得超过 `0.08 s`。
- 启用 Livox 前先建立稳定的纯 RGB-D 基线。
- 没有真实运行证据时，不得声称在线召回率或绝对定位误差通过。
