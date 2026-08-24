# fastlio_slam_merge — FAST-LIO v1 集成(建图 + 话题中继 + PCD 保存)

把 **FAST-LIO v1**(LiDAR-惯性里程计)接入 SimEnv 仿真与竞赛导航栈:`fastlio2_merge` 的
v1 前身,职责相同——把 SimEnv 的 `/scan`/IMU 转成 FAST-LIO 需要的格式,并行建 3D 点云图,
探索结束后保存**无漂移**的累积点云 PCD(锚到真值 TF)。FAST-LIO 本体在 `~/ws_livox`。

## 数据流

```
/scan (PointCloud, laser_livox)
  → pointcloud_converter.py  → /livox/Pointcloud2_local (PointCloud2)      ┐
/trunk_imu (无重力)                                                          │ FAST-LIO
  → imu_gravity_compensator.py → /trunk_imu (含重力)                        ┘ fastlio_mapping
        → /Odometry + /cloud_registered_body
  → frame_republisher.py → 重发到 odom 帧
探索结束 → save_pcd.py → 把 /cloud_registered_body 锚到真值 TF 存 PCD
```

## 关键文件

| 文件 | 作用 |
| --- | --- |
| `scripts/pointcloud_converter.py` | `/scan`(PointCloud)→ PointCloud2,FAST-LIO MARSIM 输入 |
| `scripts/imu_gravity_compensator.py` | 给 IMU 加回重力 |
| `scripts/frame_republisher.py` | 把 FAST-LIO 输出重发到 `odom` 帧 |
| `scripts/save_pcd.py` | 探索完成后把 `/cloud_registered_body`(锚到真值 TF)存 PCD |
| `config/fastlio_sim.yaml` | FAST-LIO v1 参数(含 45° 俯仰外参) |
| `launch/mapping_sim.launch` | 仅 SLAM 建图 |
| `launch/slam_nav.launch` | SLAM + 真值定位导航(并行拉 `competition_navigation`) |
| `PCD/README.md` | PCD 输出目录说明 |

## 启动

```bash
~/explorer-v1-best/scripts/start_slam_navigation.sh
# 等价: roslaunch fastlio_slam_merge slam_nav.launch
# 需同时 source ~/SimEnv 与 ~/ws_livox 两个工作空间
```

## 与其他包的关系

- 依赖 `fast_lio`(ws_livox)提供 `fastlio_mapping` 可执行。
- 是 `fastlio2_merge` 的 v1 前身;两包脚本结构一致,仅 FAST-LIO 版本不同。
