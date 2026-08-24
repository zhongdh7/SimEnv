# carto_localization — 回环矫正定位(独立功能包)

用 **Cartographer 2D 回环** 矫正 FAST-LIO 里程计在走廊退化下的漂移,产出无漂移的
`carto_map` 帧。本包**完全独立**,不修改 `competition_navigation_node.py`,不碰探索逻辑。

## 数据流

```
/scan (PointCloud, laser_livox, 10Hz, 3D +45°俯仰)
  → pointcloud_to_laserscan.py   (TF laser_livox→base, 高度过滤, 投影到水平面, 每角度取最近)
  → /scan_2d (LaserScan, frame "base")
  → cartographer_node (use_odometry=false, 纯扫描匹配 + 回环)
  → TF: carto_map → base          ← 回环矫正后的定位
  → cartographer_occupancy_grid_node → /carto_map_grid (OccupancyGrid, 供验证)
```

## 前置条件

1. 仿真已运行(Gazebo + `robot_state_publisher` + `state_from_gazebo`,即 `/scan` 与
   `base→laser_livox` 静态 TF 都存在)。**不需要** FAST-LIO 或 `ws_livox`。
2. Cartographer 已装(源码隔离安装在 `~/cartographer_ws`),`cartographer_node`、
   `cartographer_occupancy_grid_node` 可执行。本包只**引用**它,不修改它。

## 构建与安装

本仓库源码需先同步进 SimEnv 工作空间再 `catkin_make`:

```bash
export SIMENV="$HOME/SimEnv"
cd /home/loser/explorer-v1-best
./scripts/install_into_simenv.sh      # 会把 carto_localization 同步进 $SIMENV/src
cd "$SIMENV"
source /opt/ros/noetic/setup.bash
catkin_make -j2                        # 或 catkin_make --pkg carto_localization
```

## 启动

```bash
/home/loser/explorer-v1-best/scripts/start_carto_localization.sh
# 等价于:
#   source /opt/ros/noetic/setup.bash
#   source "$SIMENV/devel/setup.bash"
#   source "$HOME/cartographer_ws/install_isolated/setup.bash"
#   roslaunch carto_localization carto_locate.launch
```

带 RViz:`start_carto_localization.sh -- rviz:=true`

## 如何验证回环是否生效(核心)

1. **看日志**:`cartographer_node` 回环成功会打印
   `Adding constraint` / `Adding loop closure` 一类 INFO;全局回环搜索在
   `global_constraint_search_after_n_seconds`(默认 10s)后开始。
2. **看轨迹回位**:让机器人绕一圈回到出发点附近。用
   `rosrun tf tf_echo carto_map base` 读位姿,或 RViz 里看 `carto_map` 轨迹——
   **回环后应回到原点**;而 FAST-LIO 的 `/Odometry`(`camera_init`)会明显漂走。
3. **看地图不重影**:RViz 显示 `/carto_map_grid`,走过的同一面墙在回环前后应**重合**,
   不出现两道错开的墙(有漂移时会出现重影)。
4. **对比量化**:订阅 `/Odometry_fastlio`(漂移的)与 `carto_map→base`(矫正后的),
   绕圈回到起点后比较两者离起点的误差;carto 应远小于 fastlio。

## 预估可能出现的问题

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| `pointcloud_to_laserscan` 报 TF 查不到 | 仿真 `robot_state_publisher` 没跑 / 帧名不对 | 确认 `tf_echo base laser_livox` 有输出;默认帧名 `base`/`laser_livox` 可用 `source_frame`/`target_frame` 改 |
| `/scan_2d` 全是 inf 或几乎没有点 | 高度过滤 `min_z`/`max_z` 不对(把墙点也滤掉了) | 调 `~min_z`(默认 -0.15)、`~max_z`(默认 1.0);先 `rostopic echo /scan_2d/ranges | head` 看有没有有限值 |
| 地图出现一圈近处噪声环 | 地面点没滤干净(俯仰 45°,近处打到地面) | 把 `~min_z` 抬高(如 -0.05),或缩窄到只留墙高附近 |
| 回环不触发 / 地图重影 | `min_score` 太高或 `max_constraint_distance` 太小 | 降 `POSE_GRAPH.constraint_builder.min_score`(如 0.5)、增大 `max_constraint_distance` |
| 地图变形(墙弯了) | 纯扫描匹配在长直走廊段无回环约束时仍会轻微漂 | 属正常,回环到达后会被拉回;可增大 `optimize_every_n_nodes` 附近的回环频率 |
| 占用栅格 `/carto_map_grid` 为空 | `occupancy_grid_node` 订阅不到 submap 列表 / 未建图 | 确认 `cartographer_node` 已收到 `/scan_2d`(`rostopic hz /scan_2d` 非零) |
| 帧冲突(真值 `map→odom` 与 `carto_map` 同时存在) | 本包特意用独立帧 `carto_map`,不冲突 | 无需处理;RViz 里固定帧选 `carto_map` 即可 |

## 参数速查

- 转换节点(launch 里配):`scan_topic`、`laser_topic`、`target_frame`、`source_frame`、
  以及 `~min_z`/`~max_z`/`~range_min`/`~range_max`/`~angle_bins`。
- Cartographer:`config/carto_2d.lua`(帧名、回环 `min_score`/`max_constraint_distance`/
  `optimize_every_n_nodes`)。

## 已知限制

- 3D→2D 投影有损(45° 俯仰 + 非重复扫描),扫描匹配精度需按上表实测调参。
- 未融合 FAST-LIO/IMU(`use_odometry=false`);如需更高频输出可后续接入 `use_odometry=true`
  与帧重映射。
- 本包不接入导航(`slam_nav.launch` 仍用 `map_frame=odom`);接入属后续工作。
