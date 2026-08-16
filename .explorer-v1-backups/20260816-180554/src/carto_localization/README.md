# carto_localization

用 **Cartographer 2D** 为 SimEnv 机器人提供一个**能靠回环矫正漂移**的定位,产出无漂移的
`carto_map` 帧。

## 为什么需要它

- 导航当前定位用 Gazebo **真值里程计** `/Odometry_gazebo`(frame `odom`),真机不存在。
- 工程里真实可用的里程计是 FAST-LIO 的 `/Odometry`,但它是纯里程计、**无回环、长时间会漂**。
- 本包用 Cartographer 的**扫描匹配 + 回环**做定位,输出 `carto_map -> base`,不依赖真值、也不依赖 FAST-LIO。

## 设计要点

- **纯扫描匹配**(`use_odometry = false`,`use_imu_data = false`):只靠 2D 激光 + 回环。
- **独立帧名** `carto_map`:避免和 `state_from_gazebo` 广播的真值 `map` 帧冲突。
- **tracking frame 用 `base`**:机器人根链路是 `base`(不是 `base_link`)。
- **3D→2D 投影**:SimEnv 的 Livox Mid-360 是 3D 非重复扫描、带 45° 俯仰,发的是旧类型
  `sensor_msgs/PointCloud`(`/scan`)。`pointcloud_to_laserscan.py` 把点转到 `base` 系、
  投影到水平面、按角度分桶取最小距离,得到一帧平面 2D 的 `LaserScan`。

## 数据流

```
/scan (PointCloud, laser_livox, 10 Hz)
  -> pointcloud_to_laserscan.py   (TF laser_livox -> base, 投影 2D)
  -> /scan_2d (LaserScan, base)
  -> cartographer_node            (use_odometry=false)
  -> TF: carto_map -> base        (回环矫正后的定位)
  -> cartographer_occupancy_grid_node -> /carto_map_grid (2D 栅格图, RViz)
```

## 前置条件

1. **Cartographer 已编译**在 `~/cartographer_ws`(版本 1.0.0,`catkin_make_isolated` 安装)。
   ROS 官方 apt 已下架 `ros-noetic-cartographer`,故不能用 apt,只能用这套。
2. **SimEnv Gazebo 已运行**:`robot_state_publisher`(必须广播 `base -> laser_livox`)、
   `state_from_gazebo`、`/scan`、`/livox/imu` 都已起来。

## 编译

本仓库(`explorer-v1-best`)是**源码仓**,不是 catkin 工作空间;真正编译运行的是 `~/SimEnv`。
用同步脚本把包装进 SimEnv 再编译(`install_into_simenv.sh` 已把 `carto_localization` 加进同步列表):

```bash
~/explorer-v1-best/scripts/install_into_simenv.sh
cd ~/SimEnv
source /opt/ros/noetic/setup.bash
catkin_make -j2 --pkg carto_localization
```

## 运行

```bash
source /opt/ros/noetic/setup.bash
source ~/SimEnv/devel/setup.bash
source ~/cartographer_ws/install_isolated/setup.bash   # 关键:Cartographer
# cartographer_ws 的 setup.bash 会挤掉 SimEnv 的路径,手动合并回来:
export CMAKE_PREFIX_PATH="$HOME/SimEnv/devel:$CMAKE_PREFIX_PATH"
export ROS_PACKAGE_PATH="$HOME/SimEnv/src:$ROS_PACKAGE_PATH"
roslaunch carto_localization carto_locate.launch rviz:=true
```

## 验证

1. `rostopic echo /scan_2d` —— 有 `LaserScan` 输出,`frame_id = base`。
2. `rosrun tf tf_echo carto_map base` —— 有连续位姿。
3. RViz(`carto.rviz`)显示 `/carto_map_grid` 2D 地图 + 机器人轨迹。
4. **核心验证**:让机器人绕一圈回到起点,`carto_map -> base` 回环后应回到原点
   (对比 FAST-LIO `/Odometry` 会漂走),证明回环矫正生效。

## 已知限制 / 后续

- **3D→2D 有损**:45° 俯仰的 3D 非重复扫描转成 2D 会丢一部分信息,扫描匹配精度需实测调参
  (`config/carto_2d.lua` 里的 `min_range` / `max_range` / 匹配权重)。
- **未融合 FAST-LIO 里程计**:本版纯扫描匹配;若要「用 FAST-LIO 里程计 + 回环矫正」,需把
  `use_odometry` 打开并把 FAST-LIO 的 `camera_init -> body` 重映射成 `odom -> base`。
- **未接进导航**:导航仍用真值 `/Odometry_gazebo`;后续可把 `slam_nav.launch` 的
  `map_frame` 从 `odom` 改成 `carto_map`(只改 launch 传参,不碰探索逻辑)。
