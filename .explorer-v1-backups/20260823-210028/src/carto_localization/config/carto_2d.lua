-- Cartographer 2-D localisation with loop closure for SimEnv.
--
-- Odometry-aided 2-D localisation (use_odometry=true, use_imu_data=false):
-- odometry provides local tracking (the 45-degree-pitched lidar's degraded 2-D
-- scan cannot track translation on its own), while scan matching + loop closure
-- still correct long-term drift.  The odometry source is chosen in the launch
-- file (ground-truth /Odometry_gazebo for verification, or FAST-LIO's /Odometry
-- in production).  The result is published as TF ``carto_map -> base`` (frame
-- ``carto_map`` is used instead of ``map`` so we never clash with the
-- ground-truth ``map -> odom`` TF that state_from_gazebo already broadcasts).
--
-- The scan is /scan_2d, a LaserScan already expressed in the ``base`` frame by
-- pointcloud_to_laserscan.py, so tracking/published frame are both ``base`` and
-- Cartographer needs no extra scan-to-tracking TF.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "carto_map",
  tracking_frame = "base",
  published_frame = "base",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 30.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 25.
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 3
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- Finish submaps quickly so a short (~7 m) loop still produces several finished
-- submaps for loop closure to match against (a submap finishes at
-- 2 * num_range_data nodes; default 90 was far too slow for this robot's pace).
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 30
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
-- The 2-D scan in the featureless corridor is rotation-degenerate: scan matching
-- cannot resolve yaw, so it drifts up to ~50 deg on top of FAST-LIO2's ~5 deg.
-- Raise the rotation prior weight (default 1e-1) to match translation so the
-- scan matcher trusts the (accurate) odometry yaw instead of its own degenerate
-- estimate.  Loop closure still corrects long-term yaw globally.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 10.

-- Loop closure.  pose_graph.lua defaults are largely sane; we optimise more
-- often and extend the search distance so the building's larger revisit loops
-- still produce a constraint.
POSE_GRAPH.optimize_every_n_nodes = 45
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.min_score = 0.50
POSE_GRAPH.constraint_builder.max_constraint_distance = 30.

return options
