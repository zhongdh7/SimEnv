# competition_navigation — 原始探索导航(真值定位)

A1 仿真竞赛的**原始参考实现**:HDPlanner 驱动的全楼探索 + GBPlanner2 风格回航,
**用 Gazebo 真值里程计 `/Odometry_gazebo` 定位**。这是探索逻辑的基准版本。

> ⚠️ **约束**:不要修改本包的探索 / 房间任务 / 规划状态机(`competition_navigation_node.py`)。
> 新增功能做成独立功能包、launch 参数开关或独立节点(见 `competition_navigation_fastlio2`)。

## 它做什么

- 定位:直接订阅真值里程计 `/Odometry_gazebo`(帧 `odom`),把消息里的 pose 当机器人位姿。
- 建图:订阅 `/scan`(帧 `laser_livox`),经 TF `odom ← laser_livox` 变换后做 2D 栅格占据
  (Bresenham 光线投射),区分 UNKNOWN/FREE/OCCUPIED。
- 状态机:探索(frontier / 信息增益)→ 进房间(HDPlanner 策略推理)→ 上/下楼 → 回航。
- 输出:`/cmd_vel` 给 junior_ctrl,发布占据栅格地图与 `/competition_navigation/status`。

## 关键文件

| 文件 | 作用 |
| --- | --- |
| `scripts/competition_navigation_node.py` | 主节点:定位回调、建图、探索/房间/楼梯/回航状态机(参考实现,勿改) |
| `scripts/corridor_graph.py` | 走廊图构建辅助 |
| `src/hdplanner_inference.cpp` | HDPlanner 策略推理 C++ 库,编译成 `libhdplanner_inference.so` |
| `tools/full_map_verifier.py` | 覆盖率校验(可选启动) |
| `launch/competition_navigation.launch` | 单包启动,`odom_topic=/Odometry_gazebo`、`map_frame=odom` |

## 启动

```bash
# 单包(仅导航,需仿真 Gazebo + junior_ctrl 已运行)
~/explorer-v1-best/scripts/start_autonomous_navigation.sh
# 等价: roslaunch competition_navigation competition_navigation.launch
```

## 关键话题

- 订阅:`/Odometry_gazebo`(真值里程计)、`/scan`(PointCloud, `laser_livox`)
- 发布:`/cmd_vel`、占据栅格地图、`/competition_navigation/status`

## 与其他包的关系

- 是 `competition_navigation_fastlio2` 的**基准**(后者探索逻辑字节级相同,仅定位来源不同)。
- `fastlio_slam_merge` / `fastlio2_merge` 的 `slam_nav.launch` 会并行拉它做「真值定位导航」。
