# competition_navigation_fastlio2 — 探索导航(FAST-LIO2 定位)

与 `competition_navigation` **字节级相同的探索逻辑**,但**定位来源从 Gazebo 真值换成了
FAST-LIO2 里程计**(`/Odometry`,帧 `camera_init`)。

导航启发式不是坐标系无关的(它假设地图帧与建筑轴对齐:正门在 world x=0、走廊沿 +Y),
所以**不能**直接把 `map_frame` 设成 `camera_init`(那会让整张地图转 ~90°,机器人往左跑)。
本包用 `scan_tf_relay` 把 FAST-LIO2 位姿**重锚回建筑对齐的 `odom` 帧**,真值只在开机锚一次。

## 它做什么

- `scan_tf_relay.py`:
  1. 开机用**一次性**真值锚(等价「机器人知道起点在哪」)求出常量变换 `odom ← camera_init`;
  2. 之后每帧把 FAST-LIO2 的 `/Odometry` 变换过去 → 发 `/Odometry_fastlio`(帧 `odom`);
  3. 广播 `odom → laser_livox_fastlio`,并把 `/scan` 重贴帧后发 `/scan_fastlio`。
- 导航节点消费 `/Odometry_fastlio` + `/scan_fastlio`,`map_frame=odom`,逻辑与原包一致。
- **持续定位完全由 FAST-LIO2 提供**(漂移 ~0.3m 级),真值不再参与。

## 关键文件

| 文件 | 作用 |
| --- | --- |
| `scripts/scan_tf_relay.py` | 一次性真值锚 + 重锚定 FAST-LIO2 位姿到 odom + 扫描/帧中继 |
| `scripts/competition_navigation_node.py` | 导航主节点(逻辑与原包一致,含幽灵障碍物清除) |
| `scripts/corridor_graph.py` | 走廊图辅助 |
| `launch/competition_navigation.launch` | FAST-LIO2 定位导航(`odom_topic=/Odometry_fastlio`) |
| `launch/slam_nav.launch` | FAST-LIO2 SLAM + 本包导航 |
| `launch/carto_navigation.launch` | 用 Cartographer 回环矫正位姿(`/carto_odom`)导航 |

## 启动

```bash
# FAST-LIO2 定位导航
~/explorer-v1-best/scripts/start_slam_navigation_fastlio2_odom.sh
# 等价: roslaunch competition_navigation_fastlio2 slam_nav.launch
```

## 关键话题

- 订阅:`/Odometry`(FAST-LIO2, `camera_init`)、`/Odometry_gazebo`(仅开机锚一次)、`/scan`
- 发布:`/Odometry_fastlio`(帧 `odom`)、`/scan_fastlio`、`/cmd_vel`、栅格地图

## 与其他包的关系

- 依赖 `fastlio2_merge` / `fast_lio2`(ws_livox)提供 `/Odometry` 与 3D 建图。
- `carto_navigation.launch` 依赖 `carto_localization` 提供回环矫正后的 `/carto_odom`。
