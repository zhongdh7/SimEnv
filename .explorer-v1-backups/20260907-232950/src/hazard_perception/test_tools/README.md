# Gazebo RGB-D 危险源自动评测（正式比赛禁用）

> **高风险开发工具：** 本目录代码会读取 Gazebo 模型状态，并调用
> `/gazebo/set_model_state`、`/gazebo/spawn_sdf_model` 和 `/gazebo/delete_model`。
> 它只能在规则明确允许真值和模型操作的独立开发环境使用。正式比赛、真值无关黑盒验证、共享场景
> 联调中禁止启动，也不得把它生成的报告当作自然在线观测证据。合规的确定性正路径测试请使用
> `competition_integration/test/run_synthetic_hazard_positive_test.py`。

本目录是独立的白盒开发评测工具。评测节点可以读取 Gazebo 模型真值和 TF，但只通过
`/hazard_perception/hazards_3d` 观察正式感知节点；它不会发布真值话题，也没有被
`hazard_pipeline.launch` 包含。默认固定使用 `use_livox=false`。

## 评测内容

启动后工具会自动发现 Gazebo 中的机器人、`danger_red_sphere*` 和 CameraInfo 中的
相机 optical frame，读取 `base -> camera`、`base -> hazards output frame` TF。它保存
所有将被操作模型的原始状态，把红球依次放到相机前方配置的距离/水平角度，等待输出
稳定后采样。默认正样本为 6 个距离乘 5 个角度，共 30 个用例。

正式流水线的 tracker 会保留历史目标。评测器不要求增加生产重置接口，而是比较每个
用例最终位置稳定前后的 `observation_count`，仅统计本用例中继续收到观测的新 track。
为避免自动搬运本身触发生产 tracker 的跳变过滤，相邻最终位置之间会用若干不超过
`transition_step_m` 的 `/gazebo/set_model_state` 小步过渡；过渡观测不进入指标。成功判据
默认是：出现 confirmed 新鲜检测，且采样平均三维误差不超过 0.60 m。

配置中的红色 box/cylinder 是负样本。已有匹配模型会被临时复用；不存在时评测器通过
`/gazebo/spawn_sdf_model` 生成临时静态模型。正常结束、异常或 SIGINT 时，原有模型恢复
到精确保存的 pose，临时模型删除，不写 world、URDF、SDF 或模型文件。SIGKILL、Gazebo
崩溃或整机掉电无法执行恢复回调，遇到这种情况请重启原场景。

## 启动

先在算法 overlay 中编译目标检测包：

```bash
cd /home/lin/simenv_algorithm_ws
source /opt/ros/noetic/setup.bash
source /home/lin/SimEnv/devel/setup.bash
catkin_make --pkg hazard_perception -j2
source devel/setup.bash
```

再启动不带 Livox、带 RealSense、保持物理运行的仿真：

```bash
cd /home/lin/SimEnv
source /opt/ros/noetic/setup.bash
source devel/setup.bash

GUI=false START_CONTROLLER=0 START_BUILDING_CONTROL=0 \
  ENABLE_SENSOR_DATA=0 ENABLE_REALSENSE=1 ENABLE_LIVOX=0 \
  ENABLE_POINTCLOUD_CONVERTER=0 PAUSED=false SEED=77 \
  ROBOT_YAW=-1.5708 ./auto.sh
```

默认 competition 出生点朝建筑方向时，4–6 m 光路会被外墙遮挡；上面的 yaw 让相机朝
室外开阔区，保证距离网格是在测传感器/算法而不是墙体遮挡。若使用其他 world，请选择
前方至少 6.5 m 无遮挡的静止出生位姿；这不是手动行走要求。

另开终端启动独立评测（会同时启动纯 RGB-D 正式流水线）：

```bash
cd /home/lin/simenv_algorithm_ws
source /opt/ros/noetic/setup.bash
source /home/lin/SimEnv/devel/setup.bash
source devel/setup.bash
roslaunch src/hazard_perception/test_tools/hazard_validation.launch
```

如果正式流水线已经运行，避免节点重名：

```bash
roslaunch src/hazard_perception/test_tools/hazard_validation.launch start_pipeline:=false
```

覆盖配置和输出目录：

```bash
roslaunch src/hazard_perception/test_tools/hazard_validation.launch \
  config:=/absolute/path/custom_validation.yaml \
  output_root:=/home/lin/simenv_algorithm_ws/results/hazard_validation
```

## 输出

每次运行创建 `results/hazard_validation/run_<timestamp>/`：

- `discovery.json`：Gazebo 模型清单、实际机器人/红球、相机和输出 frame、使用的 TF；
- `summary.csv`、`summary.json`：二维成功率、三维可用率、三维定位成功率、定位
  平均/RMSE/最大误差、坐标标准差、重复目标数、误检和来源计数；
- 每个用例的 `rgb.png`、`depth_mm.png`、可选 `depth.npy`、`detection.png`；
- 每个用例的 `detections.json`（新鲜三维 track）、`two_d_detections.json`、
  `geometry_diagnostics.json` 和 `metrics.json`。

`depth_mm.png` 便于查看，`depth.npy` 保留原编码和精度。相机正前方按 ROS optical frame
定义：+X 向右、+Y 向下、+Z 向前；配置距离是相机到目标中心的精确三维斜距。默认把
目标中心向图像上方偏移 0.25 m，避免无控制器 A1 自然下沉后球体被地面裁掉；可在 YAML
中把 `target_vertical_offset_m` 改为 0。

## rosbag 录制

自动评测时另开终端录制 RGB-D 输入、TF、正式输出和调试图：

```bash
mkdir -p /home/lin/simenv_algorithm_ws/results/hazard_validation/bags
rosbag record -O /home/lin/simenv_algorithm_ws/results/hazard_validation/bags/rgbd_validation.bag \
  /clock /tf /tf_static \
  /real_sense/rgb/image_raw /real_sense/rgb/camera_info \
  /real_sense/depth/image_raw /real_sense/depth/camera_info \
  /real_sense/depth/points \
  /hazard_perception/detections \
  /hazard_perception/hazards_3d \
  /hazard_perception/geometry_diagnostics \
  /hazard_perception/pipeline_debug_image
```

## 离线回放

bag 用于离线重跑正式 RGB-D 感知，不需要也不应回放 Gazebo 真值给感知节点：

```bash
# 终端 1
roscore

# 终端 2
rosparam set use_sim_time true
rosbag play --clock --pause \
  /home/lin/SimEnv/results/hazard_validation/bags/rgbd_validation.bag

# 终端 3
cd /home/lin/SimEnv
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch hazard_perception hazard_pipeline.launch use_livox:=false
```

先启动流水线，再在播放终端按空格。为了避免与 bag 内已录制输出冲突，若要重新计算，
可用 `rosbag filter` 去掉 `/hazard_perception/*` 输出，或录制时只录输入与 TF。独立自动
摆放评测器依赖 Gazebo 的 get/set/spawn/delete 服务，因此不能只靠 bag 重新执行摆放；
离线比较应使用同一次在线运行生成的 `summary.json`、逐用例真值和回放后的输出。

## 已知限制

- 当前 RealSense Gazebo/OpenNI 深度图在约 5 m 后变为无效值；6 m 仍有正常 RGB 二维框，
  但正式流水线会正确标记 `no valid 3D`，因此 6 m 三维用例预期暴露传感器限制。
- 相机水平 FOV 约 60°，±30° 已通过独立圆弧拟合和更严格的深度几何验证覆盖；更大角度或
  可见圆弧过短时仍属于相机不可观测范围，不会通过放宽普通目标阈值强行确认。
- 生产 tracker 没有测试重置服务，评测器依靠小步搬运与 `observation_count` 增量隔离用例；
  如果自定义生产参数把允许跳变设得小于 `transition_step_m`，需要同步减小该测试参数。
- 正常退出、异常和 SIGINT 都会恢复；SIGKILL、Gazebo 崩溃或掉电无法运行恢复回调。
- 只有在线 Gazebo 能执行自动摆放；bag 可重跑感知，但不能单独重演 set/spawn/delete 服务。

## 快速检查

```bash
cd /home/lin/SimEnv
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 -m unittest \
  src/hazard_perception/test_tools/test_validation_metrics.py -v
roslaunch --nodes src/hazard_perception/test_tools/hazard_validation.launch
```
