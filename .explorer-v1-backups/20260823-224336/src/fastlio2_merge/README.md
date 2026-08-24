# fastlio2_merge — FAST-LIO2 集成(建图 + 话题中继 + PCD 保存)

把官方 **FAST-LIO2**(LiDAR-惯性里程计,直接配准,无特征提取)接入 SimEnv 仿真与竞赛导航栈:
把 SimEnv 的 `/scan`/IMU 转成 FAST-LIO2 需要的格式,并行建 3D 点云图,并在探索结束后保存
**无漂移**的累积点云 PCD(锚到真值 TF)。FAST-LIO2 本体在独立工作空间 `~/ws_livox`。

## 数据流

```
/scan (PointCloud, laser_livox)
  → pointcloud_converter.py  → /livox/Pointcloud2_local (PointCloud2 XYZI)  ┐
/trunk_imu (LinearAcceleration 无重力)                                      │ FAST-LIO2
  → imu_gravity_compensator.py → /trunk_imu_specific (含重力比力)           ┘ fastlio2_mapping
        → /Odometry (camera_init) + /cloud_registered / cloud_registered_body
  → frame_republisher.py → 把位姿/点云重发到 odom 帧
探索结束 → save_pcd.py → 把 /cloud_registered_body 锚到真值 TF 存 PCD
```

## 关键文件

| 文件 | 作用 |
| --- | --- |
| `scripts/pointcloud_converter.py` | `/scan`(PointCloud)→ PointCloud2(XYZI),FAST-LIO2 MARSIM 输入 |
| `scripts/imu_gravity_compensator.py` | 给 IMU 加回重力,转成 FAST-LIO2 期望的比力 |
| `scripts/frame_republisher.py` | 把 FAST-LIO2 输出从 `camera_init` 重发到 `odom` 帧 |
| `scripts/save_pcd.py` | 探索完成后把 `/cloud_registered_body`(锚到真值 TF)存 PCD,规避 LIO 漂移 |
| `config/fastlio2_sim.yaml` | FAST-LIO2 参数(含 45° 俯仰外参) |
| `launch/mapping_sim.launch` | 仅 SLAM 建图 |
| `launch/slam_nav.launch` | SLAM + 真值定位导航(并行拉 `competition_navigation`) |

## 启动

```bash
~/explorer-v1-best/scripts/start_slam_navigation_fastlio2.sh
# 等价: roslaunch fastlio2_merge slam_nav.launch
# 需同时 source ~/SimEnv 与 ~/ws_livox 两个工作空间
```

## 与其他包的关系

- 依赖 `fast_lio2`(ws_livox)提供 `fastlio2_mapping` 可执行。
- `slam_nav.launch` 并行拉 `competition_navigation`(真值定位导航)。
- `competition_navigation_fastlio2` 复用它的 `/Odometry` 做定位导航。
- 回环(ScanContext + GTSAM)已弃用,建图仅作探索证据;PCD 通过锚定真值 TF 消除漂移。
