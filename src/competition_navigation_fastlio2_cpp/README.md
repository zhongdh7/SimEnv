# competition_navigation_fastlio2_cpp — 探索导航（FAST-LIO2 定位，C++ 版）

`competition_navigation_fastlio2` 的 **C++ 移植**：探索逻辑与 Python 版**逐行等价**，只有语言变了，
定位来源同样是 FAST-LIO2 里程计（`/Odometry`，帧 `camera_init`），经 `scan_tf_relay` 一次性真值锚定
重投影到建筑对齐的 `odom` 帧。

把导航栈从 Python 换成 C++，是为了让网格热点循环（扫描 Bresenham 射线、A*、BFS、前沿聚类）跑得更快、
释放 CPU、提升实时确定性。HDPlanner 神经网络推理本来就是 C++（`libhdplanner_inference.so`），
这里改为运行时 `dlopen` 直接加载（等价于原版 `ctypes.CDLL`），不重复编译。

## 它做什么

与 Python 版完全一致，四个运行时节点全部 C++ 化：

| C++ 节点 | 对应 Python | 作用 |
| --- | --- | --- |
| `scan_tf_relay` | `scan_tf_relay.py` | 一次性真值锚 + FAST-LIO2 位姿重锚定到 odom + 扫描/帧中继 |
| `competition_navigation_node` | `competition_navigation_node.py` | 导航主节点（探索/房间 DFS/楼梯/返航状态机 + HDPlanner） |
| `collision_safety` | `collision_safety.py` | 安全盒 + 反向恢复 + 原地转向逃生 |
| `corridor_graph`（库） | `corridor_graph.py` | 走廊图拓扑（被主节点链接） |
| `hdplanner_policy`（库） | （ctypes 内联类） | `dlopen` 封装 HDPlanner C 桥 |

`full_map_verifier.py`（验收报告工具，非导航执行代码）仍沿用原 Python 包。

## 数据流

```
/Odometry (FAST-LIO2, camera_init) ─┐
/Odometry_gazebo (仅开机锚一次)      │ scan_tf_relay → /Odometry_fastlio (odom 帧)
/scan (laser_livox) ────────────────┘               + odom→laser_livox_fastlio TF + /scan_fastlio
        → competition_navigation_node → /cmd_vel_raw
        → collision_safety → /cmd_vel（机器人实际消费）
```

## 关键文件

| 文件 | 作用 |
| --- | --- |
| `src/competition_navigation_node.cpp` | 导航主节点（对应 3373 行 Python） |
| `src/scan_tf_relay.cpp` | FAST-LIO2 重锚定中继 |
| `src/collision_safety.cpp` | 碰撞安全层 |
| `src/corridor_graph.cpp` + `include/.../corridor_graph.h` | 走廊图 |
| `src/hdplanner_policy.cpp` + `include/.../hdplanner_policy.h` | HDPlanner dlopen 封装 |
| `launch/competition_navigation.launch` | FAST-LIO2 定位导航 |
| `launch/slam_nav.launch` | FAST-LIO2 SLAM + 本包导航 |
| `launch/carto_navigation.launch` | Cartographer 回环矫正位姿导航 |

## 启动

```bash
# 编译（需 source ~/SimEnv 与 ~/ws_livox 后 catkin_make）
# FAST-LIO2 定位导航
roslaunch competition_navigation_fastlio2_cpp slam_nav.launch
```

## 与其他包的关系

- 依赖 `fastlio2_merge` / `fast_lio2`（ws_livox）提供 `/Odometry` 与 3D 建图。
- `carto_navigation.launch` 依赖 `carto_localization` 提供回环矫正后的 `/carto_odom`。
- 是 `competition_navigation_fastlio2` 的 C++ 等价版；HDPlanner 的 `.so` 由原
  `competition_navigation` 包编译，这里仅运行时 `dlopen` 复用（避免 CUDA/libtorch 重编译与符号冲突）。

## 移植保真度说明

- 网格用 `std::vector<uint8_t>`（平铺，`y*W+x` 索引）替代 numpy，`UNKNOWN=127/FREE=255/OCCUPIED=1` 不变。
- `round()` 用 `std::lrint`（四舍六入五成双），与 Python `round()` 一致。
- 单线程 `ros::spin()` 等价于 rospy 单线程回调，原 `threading.RLock` 为无操作，故省略。
- HDPlanner 张量契约（360×7 features、K=25 edge/center 等）与 `ctypes` 版完全一致。
