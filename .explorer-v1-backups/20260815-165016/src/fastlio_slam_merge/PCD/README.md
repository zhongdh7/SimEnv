# PCD output directory

`save_pcd.py` writes the FAST-LIO accumulated point cloud here after
`competition_navigation` reports `returned_home full_map=True`.

Files are named `full_map_<sim_time>.pcd` (ASCII PCD v0.7, fields x y z and
optionally intensity).
