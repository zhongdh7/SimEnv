# competition_navigation_fastlio2 — 探索导航(FAST-LIO2 定位)

与 `competition_navigation` **字节级相同的探索逻辑**,但**定位来源从 Gazebo 真值换成了
FAST-LIO2 里程计**(`/Odometry`,帧 `camera_init`)。

导航启发式不是坐标系无关的(它假设地图帧与建筑轴对齐:正门在 world x=0、走廊沿 +Y),
所以**不能**直接把 `map_frame` 设成 `camera_init`(那会让整张地图转 ~90°,机器人往左跑)。
本包用 `scan_tf_relay` 把 FAST-LIO2 位姿**重锚回建筑对齐的 `odom` 帧**,真值只在开机锚一次。

## 关键结论(当前状态)

- ✅ **已可用 FAST-LIO2 坐标定位,不需要真值。** 导航定位完全由 FAST-LIO2 提供
  (`/Odometry_fastlio`),`/Odometry_gazebo` 真值**只在开机时锚一次**(把 `camera_init`
  对齐到建筑坐标 `odom`),之后不再读取。持续定位漂移约 **0.1–0.3 m**(沿走廊随距离
  累积,静止时收敛),真值不参与。
- ⚠️ **尚未解决:上楼梯。** 第 0 层走廊 + 4 个房间已能正常探索、进门并完成
  (`graph_rooms=4`,房间覆盖率 0.88–0.92),但**楼梯过渡(0→1→2 层上下楼)仍未能可靠完成**。

## 各种实现(定位方式)

| 实现 | 定位来源 | 启动 | 状态 |
| --- | --- | --- | --- |
| FAST-LIO2 定位导航 | `/Odometry` 重锚到 `odom` → `/Odometry_fastlio` | `slam_nav.launch` / `start_slam_navigation_fastlio2_odom.sh` | ✅ 当前使用 |
| Cartographer 回环定位 | `/carto_odom`(回环矫正,帧 `carto_map`) | `carto_navigation.launch` / `start_carto_navigation.sh` | 备用(已弃用回环栈) |
| 真值定位(原包) | `/Odometry_gazebo` | 见 `competition_navigation` 包 | 仅作对照 |

本包内部组件:

| 脚本 | 作用 |
| --- | --- |
| `scripts/scan_tf_relay.py` | 开机用**一次性**真值锚求出 `odom ← camera_init`,之后每帧重锚 FAST-LIO2 位姿到 odom,发 `/Odometry_fastlio`,并广播 `odom → laser_livox_fastlio`、重贴 `/scan` 为 `/scan_fastlio` |
| `scripts/competition_navigation_node.py` | 导航主节点(逻辑与原包一致,含幽灵障碍物清除、房间入口确认等) |
| `scripts/corridor_graph.py` | 走廊图辅助 |
| `scripts/collision_safety.py` | 独立防撞层:`/cmd_vel_raw` → `/cmd_vel`,仅当障碍**挡住机身**(正前方被占、或两侧同时被占)时清零速度;单个角落被占视为门框放行,避免进门时被门框误刹 |

## 安装到 SimEnv

前置:`$HOME/SimEnv` 是一个已存在的独立 SimEnv 工作空间(含 `auto.sh` 与 `src/`),
`ws_livox`(FAST-LIO2 C++ 本体)已单独构建。`install_into_simenv.sh` 会把本仓库的
源码 rsync 进 SimEnv(被替换文件先备份到 `$SIMENV/.explorer-v1-backups/<时间戳>/`):

```bash
export EXPLORER_ROOT="$HOME/explorer-v1-best"
export SIMENV="$HOME/SimEnv"
export HDPLANNER_ROOT="$HOME/HDPlanner_Exp_and_Nav"

cd "$EXPLORER_ROOT"
SIMENV="$SIMENV" HDPLANNER_ROOT="$HDPLANNER_ROOT" ./scripts/install_into_simenv.sh

# 重新构建 SimEnv(新增 C++ 节点/CMake 变更时才必需;纯 Python/launch 改动无需重建,
# 因为 devel/lib 下的脚本是 exec 到 src 的 relay)
cd "$SIMENV"
source /opt/ros/noetic/setup.bash
catkin_make -j2
```

说明:

- `install_into_simenv.sh` 会同步 `competition_navigation_fastlio2`、`fastlio2_merge`、
  `fastlio_slam_merge`、`carto_localization` 等包到 SimEnv,并把
  `start_slam_navigation_fastlio2_odom.sh` 等启动脚本安装到 `$SIMENV/scripts/`。
- `PCD/*.pcd` 运行输出在重装时会被保留(不会被 rsync 删除)。
- FAST-LIO2 本体(`fast_lio2` / `fastlio2_mapping`)在独立工作空间 `~/ws_livox`,
  **不**由本脚本安装;需先自行构建该工作空间。

## 启动

1. **启动仿真环境**(Gazebo + `junior_ctrl` + 机器人):

   ```bash
   export SIMENV="$HOME/SimEnv"
   cd "$SIMENV"
   source /opt/ros/noetic/setup.bash
   source devel/setup.bash
   ./auto.sh    # 可用 SEED=42 GUI=false PAUSED=true AUTO_UNPAUSE=0 等环境变量控制
   ```

   在 `junior_ctrl` 前台终端先输入 `2`(站立),确认连续的 `/Odometry_gazebo` 样本显示
   机器人站稳(高度约 0.25–0.27 m、速度/角速度接近零),再继续。

2. **启动 FAST-LIO2 建图 + 导航**(另开终端):

   ```bash
   export SIMENV="$HOME/SimEnv"
   export WSLIVOX="$HOME/ws_livox"

   ~/explorer-v1-best/scripts/start_slam_navigation_fastlio2_odom.sh
   # 等价: roslaunch competition_navigation_fastlio2 slam_nav.launch
   ```

   该脚本同时 source `SimEnv/devel` 与 `ws_livox/devel`,并显式合并两者的
   `CMAKE_PREFIX_PATH` / `ROS_PACKAGE_PATH`(两个独立 catkin 工作空间的 `setup.bash`
   会互相覆盖,顺序很重要)。`slam_nav.launch` 依次拉起:

   1. `fastlio2_merge/mapping_sim.launch` —— FAST-LIO2 建图;
   2. `competition_navigation.launch` —— FAST-LIO2 定位导航(`odom_topic=/Odometry_fastlio`,
      `map_frame=odom`,`auto_start=true`);
   3. `save_pcd.py` —— 状态出现 `returned_home full_map=True` 时保存 3D 点云,或
      `Ctrl-C` / ROS 关闭时保存已累积部分。

3. **进入 RL 导航模式**:等导航日志出现 `competition_navigation ready`,再回 `junior_ctrl`
   前台终端输入 `6`(进入 RL `/cmd_vel` 模式)。**不要在导航节点 ready 之前发送 `6`。**

4. **确认锚点正确**:`scan_tf_relay` 开机约 2 s 后应打印
   `alignment locked t_oc=(0.00, -3.20, 0.30) yaw≈90°`。若 yaw 偏到 ~-132°(或明显
   偏离 90°),说明锚点锁错(通常是因为 FAST-LIO2 在机器人走到走廊中段时才重启),需
   重置机器人到出生点再重拉导航栈。

## 关键话题

- 订阅:`/Odometry`(FAST-LIO2,`camera_init`)、`/Odometry_gazebo`(仅开机锚一次)、`/scan`
- 发布:`/Odometry_fastlio`(帧 `odom`)、`/scan_fastlio`、`/cmd_vel_raw`(经
  `collision_safety` 后发 `/cmd_vel`)、`/competition_navigation/map`(2D 占据栅格)

## 地图输出

- 3D 点云图:`save_pcd.py` 以 0.2 m 体素去重后写到
  `$SIMENV/src/fastlio2_merge/PCD/full_map_<sim_time>.pcd`(ASCII PCD v0.7;中断时文件名
  带 `_interrupted` 后缀)。这张图锚到真值 TF,不继承 LIO 漂移。
- 2D 覆盖率报告:`$SIMENV/results/full_map_coverage.json`(`full_map_verifier.py` 写)。
- 2D 占据栅格 `/competition_navigation/map` 只实时发布(RViz 可见),**不落盘**。

## 与其他包的关系

- 依赖 `fastlio2_merge` / `fast_lio2`(ws_livox)提供 `/Odometry` 与 3D 建图。
- `carto_navigation.launch` 依赖 `carto_localization` 提供回环矫正后的 `/carto_odom`
  (回环栈当前已弃用,仅作备用)。
- 原 `competition_navigation` 包是**真值定位**版本,本包只是把定位源换成 FAST-LIO2,
  探索逻辑逐字节一致。
