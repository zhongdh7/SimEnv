# odom_compare_recorder — 录制真值 vs FAST-LIO 里程计

录制 Gazebo 真值里程计 `/Odometry_gazebo`(帧 `odom→base`)与 FAST-LIO 里程计
`/Odometry`(帧 `camera_init→body`),按时间对齐写成 **CSV + rosbag**,供离线对比两个坐标系的
轨迹偏差。

## 输出

一个目录(默认 `~/odom_compare/`):

- `odom_compare.csv` — 每行一对时间对齐位姿,表头:
  `t, gt_x/y/z/qx/qy/qz/qw, lio_x/y/z/qx/qy/qz/qw`
- `odom_compare.bag` — 紧凑 rosbag(同两份里程计)

## 启动

```bash
roslaunch odom_compare_recorder record.launch
# 前提:仿真 + FAST-LIO 已在跑(两个 odom 话题都在)
```

## 关键话题

- 订阅:`/Odometry_gazebo`(真值)、`/Odometry`(FAST-LIO)

## 与其他包的关系

- 产物由 `odom_compare_player` 消费(可视化 + 漂移指标)。
