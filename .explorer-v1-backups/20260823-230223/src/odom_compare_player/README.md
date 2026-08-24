# odom_compare_player — 离线可视化 + 漂移分析

对 `odom_compare_recorder` 录制的真值 vs FAST-LIO 轨迹做**离线可视化与漂移分析**:
RViz 静态叠加(绿=真值、红=FAST-LIO、蓝=偏差连线)、动画回放,以及 ATE / RPE / 航向的
matplotlib 指标图。

## 组件

| 文件 | 作用 |
| --- | --- |
| `scripts/visualizer.py` | 静态叠加:Umeyama 刚体对齐后发 gt/lio/deviation 三条 marker(周期重发) |
| `scripts/animate.py` | 动画回放:逐步生长轨迹 + 位姿 |
| `scripts/analyze.py` | 独立运行(无需 ROS master)的 ATE / RPE / 航向漂移指标 + PNG |
| `scripts/odom_compare_lib.py` | 公共:CSV 读取、Umeyama 对齐、指标计算 |
| `launch/player.launch` | 静态 + 动画 + RViz 一键起 |
| `rviz/odom_compare.rviz` | RViz 配置(Fixed Frame `map`,静态 TF `map→world`) |

## 启动

```bash
# 静态叠加 + 动画回放 + RViz
roslaunch odom_compare_player player.launch \
    csv_path:=~/odom_compare/odom_compare.csv animate:=true
# 仅漂移指标(无需 ROS master)
python3 scripts/analyze.py --csv ~/odom_compare/odom_compare.csv
```

## 关键话题

- 发布:`/odom_compare_markers`(gt/lio/deviation)、`/gt_path_anim`、`/lio_path_anim`、
  `/gt_pose`、`/lio_pose`

## 与其他包的关系

- 消费 `odom_compare_recorder` 的 CSV。
