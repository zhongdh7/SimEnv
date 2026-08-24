#!/usr/bin/env python3
"""ROS1 adapter for HDPlanner-Nav and GBPlanner2-style homing.

The simulator exposes a Livox PointCloud and Gazebo odometry, while the
controller consumes geometry_msgs/Twist on /cmd_vel.  This node keeps that
adapter deliberately small: a 2-D belief map, a deterministic HDPlanner
policy adapter when the checkpoint is available, and an A* return path.
"""

from __future__ import print_function

import ctypes
import heapq
import math
import os
import threading
import traceback
from collections import deque

import numpy as np
import rospy
import tf
import tf.transformations as transformations
from geometry_msgs.msg import Point32, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

from corridor_graph import CorridorGraph


UNKNOWN = 127
FREE = 255
OCCUPIED = 1


# The v1 competition scene fixes these infrastructure values.  The scene
# seed changes room contents and obstacle placement, not the navigation
# geometry used by this node.
DEFAULT_FLOOR_COUNT = 3
DEFAULT_ROOMS_PER_FLOOR = 4
DEFAULT_FLOOR_HEIGHT = 2.6
DEFAULT_SCAN_MIN_RANGE = 2.0
DEFAULT_SCAN_SELF_FILTER_ENABLED = True
DEFAULT_SCAN_SELF_FILTER_FRONT = 0.75
DEFAULT_SCAN_SELF_FILTER_REAR = 0.60
DEFAULT_SCAN_SELF_FILTER_LEFT = 0.55
DEFAULT_SCAN_SELF_FILTER_RIGHT = 0.55
DEFAULT_FOOTPRINT_BOUNDS = {
    "x_min": -10.0,
    "x_max": 10.0,
    "y_min": 0.0,
    "y_max": 36.0,
}
DEFAULT_LOBBY_BOUNDS = {
    "x_min": -10.0,
    "x_max": 10.0,
    "y_min": 0.0,
    "y_max": 7.85,
}
DEFAULT_CORRIDOR_BOUNDS = {
    "x_min": -1.1,
    "x_max": 1.1,
    "y_min": 7.85,
    "y_max": 35.91,
}
DEFAULT_STAIR_BOUNDS = {
    "x_min": -4.85,
    "x_max": -1.65,
    "y_min": 0.85,
    "y_max": 6.85,
}
DEFAULT_ELEVATOR_BOUNDS = {
    "x_min": 1.65,
    "x_max": 4.05,
    "y_min": 1.25,
    "y_max": 3.95,
}
def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def yaw_from_quaternion(q):
    return transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]


def bresenham(x0, y0, x1, y1):
    """Yield integer cells on a line, including both endpoints."""
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    x, y = x0, y0
    while True:
        yield x, y
        if x == x1 and y == y1:
            return
        twice_error = 2 * error
        if twice_error >= dy:
            error += dy
            x += sx
        if twice_error <= dx:
            error += dx
            y += sy


class HDPlannerPolicy(object):
    """Runs the published HDPlanner PolicyNet through a LibTorch C bridge."""

    NODE_COUNT = 360
    FEATURE_COUNT = 7
    K_SIZE = 25

    def __init__(self, library_path, model_path):
        self.ready = False
        self.reason = "disabled"
        self.library = None
        self.handle = None
        self.inference_count = 0
        self.last_selected_center = -1
        self.last_action_logp = float("nan")
        try:
            self.library = ctypes.CDLL(library_path)
            float_array = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
            int64_array = np.ctypeslib.ndpointer(dtype=np.int64, flags="C_CONTIGUOUS")
            int16_array = np.ctypeslib.ndpointer(dtype=np.int16, flags="C_CONTIGUOUS")
            self.library.hdplanner_create.argtypes = [
                ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int64
            ]
            self.library.hdplanner_create.restype = ctypes.c_void_p
            self.library.hdplanner_destroy.argtypes = [ctypes.c_void_p]
            self.library.hdplanner_destroy.restype = None
            self.library.hdplanner_select.argtypes = [
                ctypes.c_void_p,
                float_array,
                int64_array,
                ctypes.c_int64,
                ctypes.c_int64,
                int64_array,
                int16_array,
                int16_array,
                int64_array,
                int64_array,
                ctypes.POINTER(ctypes.c_int64),
                ctypes.POINTER(ctypes.c_int64),
                ctypes.POINTER(ctypes.c_float),
                ctypes.c_char_p,
                ctypes.c_int64,
            ]
            self.library.hdplanner_select.restype = ctypes.c_int
            error = ctypes.create_string_buffer(4096)
            self.handle = self.library.hdplanner_create(
                os.fsencode(model_path), error, len(error)
            )
            if not self.handle:
                raise RuntimeError(error.value.decode("utf-8", "replace"))
            self.ready = True
            self.reason = "torchscript_checkpoint_loaded"
            rospy.loginfo("HDPlanner TorchScript policy loaded from %s", model_path)
        except Exception as exc:
            self.close()
            self.reason = "%s: %s" % (type(exc).__name__, exc)
            rospy.logerr("HDPlanner policy unavailable: %s", self.reason)

    def close(self):
        if self.library is not None and self.handle:
            self.library.hdplanner_destroy(self.handle)
        self.handle = None
        self.ready = False

    def select(self, nodes, utility, visited, current_index, target_index,
               neighbors, centers, adjacency):
        """Return a neighboring node using the checkpoint's original tensor contract."""
        node_count = len(nodes)
        if not self.ready or not neighbors or node_count >= self.NODE_COUNT:
            return None

        current = nodes[current_index]
        target = nodes[target_index]
        features = np.zeros((self.NODE_COUNT, self.FEATURE_COUNT), dtype=np.float32)
        center_set = set(centers)
        for index, node in enumerate(nodes):
            features[index, 0:2] = (node - current) / 60.0
            features[index, 2] = 1.0 if utility[index] > 0 else 0.0
            features[index, 3] = 1.0 if visited[index] else 0.0
            features[index, 4:6] = (target - current) / 60.0
            features[index, 6] = 1.0 if index in center_set else 0.0

        edge_inputs = np.full((self.K_SIZE,), current_index, dtype=np.int64)
        edge_padding = np.ones((self.K_SIZE,), dtype=np.int16)
        valid_neighbors = [index for index in neighbors if index != current_index]
        for offset, index in enumerate(valid_neighbors[:self.K_SIZE - 1], start=1):
            edge_inputs[offset] = index
            edge_padding[offset] = 0
        edge_padding[0] = 1

        center_indices = list(centers[:self.K_SIZE]) or [target_index]
        center_inputs = np.full((self.K_SIZE,), self.NODE_COUNT - 1, dtype=np.int64)
        center_padding = np.ones((self.K_SIZE,), dtype=np.int64)
        for offset, index in enumerate(center_indices):
            center_inputs[offset] = index
            center_padding[offset] = 0

        node_padding = np.ones((self.NODE_COUNT,), dtype=np.int16)
        node_padding[:node_count] = 0
        edge_mask = np.ones((self.NODE_COUNT, self.NODE_COUNT), dtype=np.int64)
        for index, adjacent in enumerate(adjacency):
            edge_mask[index, index] = 0
            for neighbor in adjacent:
                edge_mask[index, neighbor] = 0

        arrays = (features, edge_inputs, center_inputs, node_padding,
                  edge_padding, edge_mask, center_padding)
        if not all(np.isfinite(value).all() for value in (features,)):
            rospy.logerr_throttle(5.0, "HDPlanner observation contains NaN/Inf")
            return None
        arrays = tuple(np.ascontiguousarray(value) for value in arrays)
        (features, edge_inputs, center_inputs, node_padding,
         edge_padding, edge_mask, center_padding) = arrays
        selected = ctypes.c_int64(-1)
        selected_center = ctypes.c_int64(-1)
        action_logp = ctypes.c_float(float("nan"))
        error = ctypes.create_string_buffer(4096)
        result = self.library.hdplanner_select(
            self.handle, features, edge_inputs, current_index, target_index,
            center_inputs, node_padding, edge_padding, edge_mask, center_padding,
            ctypes.byref(selected), ctypes.byref(selected_center),
            ctypes.byref(action_logp), error, len(error)
        )
        if result != 0:
            rospy.logerr_throttle(
                5.0, "HDPlanner inference failed (%d): %s",
                result, error.value.decode("utf-8", "replace")
            )
            return None
        selected_index = int(selected.value)
        if selected_index not in valid_neighbors:
            rospy.logerr_throttle(
                5.0, "HDPlanner selected non-neighbor node %d", selected_index
            )
            return None
        if not math.isfinite(float(action_logp.value)):
            rospy.logerr_throttle(5.0, "HDPlanner action contains NaN/Inf")
            return None
        self.inference_count += 1
        self.last_selected_center = int(selected_center.value)
        self.last_action_logp = float(action_logp.value)
        return selected_index


class CompetitionNavigation(object):
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "odom")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry_gazebo")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.cell_size = float(rospy.get_param("~cell_size", 0.4))
        self.map_size_m = float(rospy.get_param("~map_size_m", 80.0))
        self.planning_period = float(rospy.get_param("~planning_period", 2.0))
        self.control_period = float(rospy.get_param("~control_period", 0.1))
        # Rate at which the occupancy-grid map (and status) is re-published.
        # The belief array itself is updated on every scan (~10 Hz), so this
        # only controls how often the result is pushed out for visualisation /
        # downstream consumers.  During in-place turns the robot sweeps new
        # cells quickly, so a higher rate makes the map look much more
        # responsive.
        self.map_publish_rate = max(
            1.0, float(rospy.get_param("~map_publish_rate", 5.0))
        )
        # Anti-re-exploration: a frontier target is skipped if it lies within
        # this many metres of any position the robot has already visited on the
        # current floor.  Occluded frontiers in already-explored rooms otherwise
        # keep winning the information-gain score and waste time revisiting.
        self.frontier_revisit_radius = max(
            0.0, float(rospy.get_param("~frontier_revisit_radius", 0.8))
        )
        self.frontier_revisit_radius_cells = max(
            0, int(math.ceil(self.frontier_revisit_radius / self.cell_size))
        )
        self.scan_stride = max(1, int(rospy.get_param("~scan_stride", 8)))
        self.max_scan_points = max(100, int(rospy.get_param("~max_scan_points", 1800)))
        self.scan_min_range = max(
            0.0, float(rospy.get_param("~scan_min_range", DEFAULT_SCAN_MIN_RANGE))
        )
        self.scan_max_range = max(
            self.scan_min_range + 0.5,
            float(rospy.get_param("~scan_max_range", 22.0)),
        )
        self.scan_self_filter_enabled = bool(
            rospy.get_param(
                "~scan_self_filter_enabled", DEFAULT_SCAN_SELF_FILTER_ENABLED
            )
        )
        self.scan_self_filter_front = max(
            0.0,
            float(rospy.get_param(
                "~scan_self_filter_front", DEFAULT_SCAN_SELF_FILTER_FRONT
            )),
        )
        self.scan_self_filter_rear = max(
            0.0,
            float(rospy.get_param(
                "~scan_self_filter_rear", DEFAULT_SCAN_SELF_FILTER_REAR
            )),
        )
        self.scan_self_filter_left = max(
            0.0,
            float(rospy.get_param(
                "~scan_self_filter_left", DEFAULT_SCAN_SELF_FILTER_LEFT
            )),
        )
        self.scan_self_filter_right = max(
            0.0,
            float(rospy.get_param(
                "~scan_self_filter_right", DEFAULT_SCAN_SELF_FILTER_RIGHT
            )),
        )
        self.map_progress_cell_batch = max(
            1, int(rospy.get_param("~map_progress_cell_batch", 8))
        )
        # Percentage-based ghost-obstacle clearing (see scan_callback).  An
        # occupied cell is cleared back to free only once the fraction of rays
        # that pass *through* it (vs. end on it) exceeds this percent, and only
        # after at least obstacle_clear_min_obs observations.  A real wall keeps
        # getting hit so its free-pass fraction stays low and it survives; a
        # spurious 3-D-flattening point gets mostly free passes and is erased.
        # Raise the percent to be more conservative (walls safer), lower it to
        # drop ghost points more aggressively.
        self.obstacle_clear_percent = float(
            rospy.get_param("~obstacle_clear_percent", 80.0)
        )
        self.obstacle_clear_min_obs = max(
            1, int(rospy.get_param("~obstacle_clear_min_obs", 5))
        )
        self.max_linear = float(rospy.get_param("~max_linear", 0.5))
        self.linear_gain = float(rospy.get_param("~linear_gain", 1.0))
        self.max_lateral = float(rospy.get_param("~max_lateral", 0.15))
        self.max_yaw_rate = float(rospy.get_param("~max_yaw_rate", 1.2))
        # Robot footprint radius (Unitree A1: trunk ~0.194 m wide x ~0.267 m
        # long, legs out to ~0.4 m total => ~0.20 m half-width).  The planner
        # treats the robot as a disc of this radius, so even unknown-space
        # paths (doorway + room interior) keep the *centre* at least this far
        # from any occupied cell.  A point-planner otherwise routes the centre
        # straight against a door frame and the ~0.4 m body rubs/hooks the
        # frame while entering.  Capped implicitly by the 0.8 m room door: this
        # radius leaves a centred ~0.4 m passage through it.
        self.robot_radius = max(0.0, float(rospy.get_param("~robot_radius", 0.20)))
        self.robot_radius_cells = (
            0 if self.robot_radius <= 0.0
            else max(1, int(math.ceil(self.robot_radius / self.cell_size)))
        )
        self.allow_unknown_return = bool(rospy.get_param("~allow_unknown_return", False))
        self.auto_start = bool(rospy.get_param("~auto_start", False))
        self.auto_return_after_sec = float(rospy.get_param("~auto_return_after_sec", 0.0))
        self.require_hdplanner = bool(rospy.get_param("~require_hdplanner", True))
        self.floor_count = max(
            1, int(rospy.get_param("~floor_count", DEFAULT_FLOOR_COUNT))
        )
        self.floor_height = float(
            rospy.get_param("~floor_height", DEFAULT_FLOOR_HEIGHT)
        )
        self.min_floor_coverage = float(rospy.get_param("~min_floor_coverage", 0.45))
        self.completion_min_floor_coverage = max(
            self.min_floor_coverage,
            float(rospy.get_param("~completion_min_floor_coverage", 0.70)),
        )
        self.completion_min_sector_coverage = float(
            rospy.get_param("~completion_min_sector_coverage", 0.30)
        )
        self.completion_min_floor_distance = float(
            rospy.get_param("~completion_min_floor_distance", 35.0)
        )
        self.completion_cycles_required = max(
            2, int(rospy.get_param("~completion_cycles_required", 3))
        )
        self.no_frontier_cycles_required = max(
            2, int(rospy.get_param("~no_frontier_cycles_required", 6))
        )
        self.stair_speed = max(
            0.05, float(rospy.get_param("~stair_speed", 0.80))
        )
        self.stair_landing_speed = max(
            0.0, float(rospy.get_param("~stair_landing_speed", 0.40))
        )
        self.stair_yaw_rate = max(
            0.05, float(rospy.get_param("~stair_yaw_rate", 1.0))
        )
        self.stair_flat_use_max_linear = bool(
            rospy.get_param("~stair_flat_use_max_linear", True)
        )
        self.stair_lookahead = max(
            0.05, float(rospy.get_param("~stair_lookahead", 0.30))
        )
        self.stair_brake_distance = max(
            self.stair_lookahead,
            float(rospy.get_param("~stair_brake_distance", 0.35)),
        )
        self.stair_lateral_limit = max(
            0.03, float(rospy.get_param("~stair_lateral_limit", 0.08))
        )
        self.stair_cruise_heading_tolerance = float(
            rospy.get_param("~stair_cruise_heading_tolerance", 0.70)
        )
        self.stair_cruise_lateral_tolerance = float(
            rospy.get_param("~stair_cruise_lateral_tolerance", 0.22)
        )
        self.stair_turn_heading_threshold = float(
            rospy.get_param("~stair_turn_heading_threshold", 0.60)
        )
        self.min_room_coverage = max(
            0.50, float(rospy.get_param("~min_room_coverage", 0.70))
        )
        self.room_task_timeout = max(
            30.0, float(rospy.get_param("~room_task_timeout", 120.0))
        )
        # Room-local exploration is deliberately independent from HDPlanner.
        # These parameters keep the first implementation easy to tune in roslaunch.
        self.room_frontier_min_cluster_cells = max(
            1, int(rospy.get_param("~room_frontier_min_cluster_cells", 3))
        )
        self.room_candidates_per_cluster = max(
            1, int(rospy.get_param("~room_candidates_per_cluster", 2))
        )
        self.room_candidate_limit = max(
            1, int(rospy.get_param("~room_candidate_limit", 10))
        )
        self.room_information_range = max(
            self.cell_size,
            float(rospy.get_param("~room_information_range", 6.0)),
        )
        self.room_information_angle_step_deg = clamp(
            float(rospy.get_param("~room_information_angle_step_deg", 10.0)),
            2.0,
            45.0,
        )
        self.room_path_cost_weight = max(
            0.0, float(rospy.get_param("~room_path_cost_weight", 0.45))
        )
        self.room_return_cost_weight = max(
            0.0, float(rospy.get_param("~room_return_cost_weight", 0.20))
        )
        self.room_obstacle_cost_weight = max(
            0.0, float(rospy.get_param("~room_obstacle_cost_weight", 0.80))
        )
        self.room_target_blacklist_radius = max(
            0.0,
            float(rospy.get_param("~room_target_blacklist_radius", 0.80)),
        )
        self.room_entry_confirmation_cycles = max(
            1, int(rospy.get_param("~room_entry_confirmation_cycles", 2))
        )
        self.room_entry_allow_unknown_wall = bool(
            rospy.get_param("~room_entry_allow_unknown_wall", True)
        )
        # Minimum doorway width (in cells) required to accept a side opening.
        # Measured: real doors are 0.8-1.2 m (4-6 cells) while drift-induced
        # phantom openings are 0.2-0.4 m (1-2 cells), so 3 cells cleanly
        # separates them.
        self.room_entry_min_width_cells = max(
            2, int(rospy.get_param("~room_entry_min_width_cells", 3))
        )
        # Phantom-door wall-support filter: a real doorway sits in a SOLID
        # wall -- at least one side (above or below the opening, along the
        # wall line) keeps continuous OCCUPIED wall, and the opening itself is
        # narrow (0.6-1.2 m).  A drift/observation-broken wall segment (e.g.
        # the 1st-floor far-corridor left wall) instead shows either a very
        # WIDE free gap (6-8+ cells) or FREE cells on both sides.
        # ``room_entry_wall_support_cells`` is the wall window checked on each
        # side (in cells); ``room_entry_wall_support_min_occupied`` is how many
        # of those cells must be OCCUPIED on at least one side; and
        # ``room_entry_max_width_cells`` caps the opening width.  Both checks
        # are applied only when the values are > 0.
        self.room_entry_wall_support_cells = max(
            0, int(rospy.get_param("~room_entry_wall_support_cells", 3))
        )
        self.room_entry_wall_support_min_occupied = max(
            1, int(rospy.get_param("~room_entry_wall_support_min_occupied", 2))
        )
        self.room_entry_max_width_cells = max(
            0, int(rospy.get_param("~room_entry_max_width_cells", 7))
        )
        # Fast-fail guard: while trying to ENTER a room, count consecutive
        # planning cycles where no entry path could be computed.  A phantom
        # doorway has no real room behind it, so the entry A* never succeeds
        # and the old code waited the full room_task_timeout (120 s) before
        # giving up.  Fail fast after this many consecutive empty-path
        # planning cycles (default 5 * planning_period 2 s = ~10 s) and mark
        # the room blocked so DFS moves on.
        self.room_entry_path_fail_limit = max(
            1, int(rospy.get_param("~room_entry_path_fail_limit", 5))
        )
        rooms_per_floor = max(
            1,
            int(rospy.get_param("~rooms_per_floor", DEFAULT_ROOMS_PER_FLOOR)),
        )
        self.stair_bounds = dict(DEFAULT_STAIR_BOUNDS)
        self.corridor_bounds = dict(DEFAULT_CORRIDOR_BOUNDS)
        # Populated from observed room_entry nodes in the lidar/odometry map.
        self.room_door_ys = []
        self.room_entry_observations = [dict() for _ in range(self.floor_count)]
        self.expected_room_count = [rooms_per_floor for _ in range(self.floor_count)]
        self.footprint_bounds = dict(DEFAULT_FOOTPRINT_BOUNDS)
        self.lobby_bounds = dict(DEFAULT_LOBBY_BOUNDS)
        self.elevator_bounds = dict(DEFAULT_ELEVATOR_BOUNDS)
        self.model_path = rospy.get_param(
            "~model_path",
            "/home/uf/HDPlanner_Exp_and_Nav/model/HDPlanner_Nav/policy_traced.pt",
        )
        self.library_path = rospy.get_param(
            "~library_path",
            "/home/uf/SimEnv/devel/lib/libhdplanner_inference.so",
        )

        cells = int(round(self.map_size_m / self.cell_size))
        if cells < 80 or cells > 600:
            raise ValueError("map_size_m must produce 80..600 cells")
        self.map_cells = cells
        self.floor_beliefs = [
            np.full((cells, cells), UNKNOWN, dtype=np.uint8)
            for _ in range(self.floor_count)
        ]
        # Per-cell hit / free-pass counts for percentage-based clearing of
        # ghost obstacles (see obstacle_clear_percent).  Only occupied cells
        # carry meaningful values; both counters reset on any state change.
        self.obs_hits = [
            np.zeros((cells, cells), dtype=np.int32)
            for _ in range(self.floor_count)
        ]
        self.obs_free = [
            np.zeros((cells, cells), dtype=np.int32)
            for _ in range(self.floor_count)
        ]
        self._reset_graph_state()
        self.map_origin_x = None
        self.map_origin_y = None
        self.exploration_roi = None

        self.lock = threading.RLock()
        self.tf_listener = tf.TransformListener(cache_time=rospy.Duration(10.0))
        self.pose = None
        self.home = None
        self.current_floor = 0
        self.home_floor = 0
        self.visited_by_floor = [set() for _ in range(self.floor_count)]
        self.visited_cells = [set() for _ in range(self.floor_count)]
        self.floor_complete = [False for _ in range(self.floor_count)]
        self.no_frontier_cycles = [0 for _ in range(self.floor_count)]
        self.distributed_coverage_cycles = [0 for _ in range(self.floor_count)]
        self.reachable_frontiers = [0 for _ in range(self.floor_count)]
        self.floor_coverage = [0.0 for _ in range(self.floor_count)]
        self.floor_distance = [0.0 for _ in range(self.floor_count)]
        self.last_distance_pose = None
        self.transition_target_floor = None
        self.return_after_transition = False
        self.full_map_complete = False
        self.current_frontier_cell = None
        self.frontier_progress_cell = None
        self.frontier_best_distance = float("inf")
        self.frontier_progress_time = rospy.Time(0)
        self.blacklisted_frontiers = [set() for _ in range(self.floor_count)]
        self.last_progress_pose = None
        self.last_progress_time = rospy.Time(0)
        # Generic path-progress stall tracking (control timer).  When the robot
        # follows a path but makes no pose progress for a while -- e.g.
        # collision_safety is holding it against an obstacle -- drop the path
        # so the planning timer re-plans instead of pushing into the obstacle
        # forever (measured: collision trip at floor-0 corridor ghost, robot
        # frozen while nav kept sending full-speed forward).
        self.ctrl_last_pose = None
        self.ctrl_last_time = rospy.Time(0)
        self.ctrl_stall_timeout = max(
            4.0, float(rospy.get_param("~ctrl_stall_timeout", 8.0))
        )
        self.pending_known_cells = [0 for _ in range(self.floor_count)]
        self.latest_scan_stamp = rospy.Time(0)
        self.last_map_update = rospy.Time(0)
        self.active = False
        self.mode = "idle"
        self.path = []
        self.path_index = 0
        self.last_plan = rospy.Time(0)
        self.last_command = Twist()
        self.started_at = None
        self.status = "waiting_for_odom"
        self.policy = HDPlannerPolicy(self.library_path, self.model_path)
        if self.require_hdplanner and not self.policy.ready:
            raise RuntimeError("required HDPlanner backend failed: %s" % self.policy.reason)

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.map_pub = rospy.Publisher("/competition_navigation/map", OccupancyGrid, queue_size=1, latch=True)
        self.floor_map_pubs = [
            rospy.Publisher(
                "/competition_navigation/map_floor_%d" % floor,
                OccupancyGrid,
                queue_size=1,
                latch=True,
            )
            for floor in range(self.floor_count)
        ]
        self.waypoint_pub = rospy.Publisher("/competition_navigation/waypoint", PoseStamped, queue_size=1)
        self.status_pub = rospy.Publisher("/competition_navigation/status", String, queue_size=2)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=5)
        rospy.Subscriber(self.scan_topic, PointCloud, self.scan_callback, queue_size=1)
        rospy.Service("/competition_navigation/start", Trigger, self.start_callback)
        rospy.Service("/competition_navigation/return_home", Trigger, self.return_home_callback)
        rospy.Service("/competition_navigation/set_home", Trigger, self.set_home_callback)
        rospy.Service("/competition_navigation/stop", Trigger, self.stop_callback)
        rospy.Timer(rospy.Duration(self.control_period), self.control_timer)
        rospy.Timer(rospy.Duration(self.planning_period), self.planning_timer)
        rospy.Timer(
            rospy.Duration(1.0 / self.map_publish_rate), self.publish_timer
        )
        rospy.on_shutdown(self.shutdown)

        if self.auto_start:
            rospy.logwarn("auto_start enabled: controller must already be in RL mode 6")
            self.active = True
            self.mode = "explore"
            self.started_at = rospy.Time.now()

        rospy.loginfo(
            "competition_navigation ready: scan=%s odom=%s cmd_vel=%s map=%dx%d backend=%s",
            self.scan_topic,
            self.odom_topic,
            self.cmd_vel_topic,
            cells,
            cells,
            self.policy.reason,
        )

    def shutdown(self):
        self.publish_zero()
        self.policy.close()

    def _reset_graph_state(self):
        self.floor_graphs = [CorridorGraph(merge_distance=0.5)
                             for _ in range(self.floor_count)]
        self.graph_root_nodes = [None for _ in range(self.floor_count)]
        self.graph_anchor_nodes = [dict() for _ in range(self.floor_count)]
        self.graph_end_nodes = [None for _ in range(self.floor_count)]
        self.graph_phase = ["corridor_discovery" for _ in range(self.floor_count)]
        self.graph_dfs_order = [[] for _ in range(self.floor_count)]
        self.graph_dfs_index = [0 for _ in range(self.floor_count)]
        self.graph_dfs_attempts = [0 for _ in range(self.floor_count)]
        self.active_room_node = None
        self.active_room_phase = None
        self.active_room_entered = False
        self.active_room_started = rospy.Time(0)
        self.active_room_no_frontier_cycles = 0
        self.active_room_target_cell = None
        self.active_room_frontier_blacklist = set()
        self.room_entry_path_failures = 0
        self.last_room_plan = None
        # Room-task stall recovery: a doorway/return path can sit at
        # path_index < len(path) forever when the next waypoint is
        # unreachable (a drift-induced phantom doorway, or the body wedged on
        # a door frame).  The state machine's path-following branch returns
        # True indefinitely and room_task_timeout only covers the *empty*
        # path case, so without this the robot never abandons such a room.
        # Track the last pose that registered meaningful motion and flag a
        # stall once neither position nor heading has advanced for a timeout.
        self.room_entry_progress_pose = None
        self.room_entry_progress_time = rospy.Time(0)
        self.room_entry_stall_timeout = max(
            3.0, float(rospy.get_param("~room_entry_stall_timeout", 10.0))
        )
        self.room_door_ys_by_floor = [[] for _ in range(self.floor_count)]
        self.room_entry_observations = [dict() for _ in range(self.floor_count)]

    def _graph_root(self, floor_index):
        graph = self.floor_graphs[floor_index]
        root_id = self.graph_root_nodes[floor_index]
        if root_id is not None:
            return root_id
        corridor = self.corridor_bounds or {}
        y = float(corridor.get("y_min", 0.0))
        x = 0.0
        root_id, _ = graph.add_node(
            "entrance", floor_index, x, y, corridor_s=0.0,
            confidence=1.0, label="corridor_entrance",
        )
        self.graph_root_nodes[floor_index] = root_id
        return root_id

    def _room_entry_candidates(self, belief):
        """Find multi-cell side openings in the observed corridor walls."""
        corridor = self.corridor_bounds
        if not corridor or self.map_origin_x is None:
            return []
        x_min = float(corridor["x_min"])
        x_max = float(corridor["x_max"])
        y_min = float(corridor["y_min"]) + 0.8
        y_max = float(corridor["y_max"]) - 0.8
        candidates = []
        for side, wall_x, room_direction in (
                ("left", x_min, -1.0), ("right", x_max, 1.0)):
            samples = []
            y = y_min
            while y <= y_max:
                wall_cell = self._cell((wall_x, y))
                room_cell = self._cell((wall_x + room_direction * 0.45, y))
                corridor_cell = self._cell((wall_x - room_direction * 0.45, y))
                opening = False
                if (wall_cell is not None and room_cell is not None and
                        corridor_cell is not None):
                    wall_value = belief[wall_cell[1], wall_cell[0]]
                    wall_open = (
                        wall_value == FREE or
                        (self.room_entry_allow_unknown_wall and
                         wall_value == UNKNOWN)
                    )
                    opening = (
                        wall_open and
                        belief[room_cell[1], room_cell[0]] == FREE and
                        belief[corridor_cell[1], corridor_cell[0]] == FREE
                    )
                samples.append((y, opening))
                y += self.cell_size
            start = None
            for index, (_, opening) in enumerate(samples + [(None, False)]):
                if opening and start is None:
                    start = index
                if not opening and start is not None:
                    run = samples[start:index]
                    if len(run) >= self.room_entry_min_width_cells:
                        # Phantom-door filter (measured on the 1st-floor
                        # far-corridor left wall): a real doorway is narrow
                        # (<= room_entry_max_width_cells) and has at least one
                        # side with a solid wall fragment
                        # (>= room_entry_wall_support_min_occupied OCCUPIED
                        # cells within room_entry_wall_support_cells along the
                        # wall line); a broken-wall phantom opening is either
                        # very wide or has FREE cells on both sides.
                        width_ok = True
                        if (self.room_entry_max_width_cells > 0 and
                                len(run) > self.room_entry_max_width_cells):
                            width_ok = False
                        support_ok = True
                        if (width_ok and
                                self.room_entry_wall_support_cells > 0):
                            support_ok = False
                            window = self.room_entry_wall_support_cells
                            for item_list, step in (
                                    (samples, -1), (samples, 1)):
                                base = start if step < 0 else index - 1
                                occupied_count = 0
                                checked = 0
                                for k in range(1, window + 1):
                                    pos = base + step * k
                                    if pos < 0 or pos >= len(samples):
                                        break
                                    wall_cell = self._cell(
                                        (wall_x, samples[pos][0])
                                    )
                                    checked += 1
                                    if (wall_cell is not None and
                                            belief[wall_cell[1], wall_cell[0]] == OCCUPIED):
                                        occupied_count += 1
                                if (checked >= 1 and occupied_count >=
                                        self.room_entry_wall_support_min_occupied):
                                    support_ok = True
                                    break
                        if width_ok and support_ok:
                            center_y = sum(item[0] for item in run) / len(run)
                            candidates.append({
                                "side": side,
                                "x": wall_x + room_direction * 0.60,
                                "y": center_y,
                                "yaw": math.pi if side == "left" else 0.0,
                                "corridor_s": center_y - float(corridor["y_min"]),
                                "confidence": min(1.0, len(run) / 4.0),
                            })
                    start = None
        return candidates

    def _confirmed_room_entry_candidates(self, candidates, floor_index):
        """Require a door opening to persist across planning observations."""
        if floor_index >= len(self.room_entry_observations):
            return []
        previous = self.room_entry_observations[floor_index]
        current = {}
        confirmed = []
        for candidate in candidates:
            key = (
                candidate["side"],
                int(round(float(candidate["y"]) / max(self.cell_size, 0.1))),
            )
            count = min(
                self.room_entry_confirmation_cycles,
                int(previous.get(key, 0)) + 1,
            )
            current[key] = count
            if count >= self.room_entry_confirmation_cycles:
                confirmed_candidate = dict(candidate)
                confirmed_candidate["confidence"] = min(
                    1.0,
                    max(float(candidate.get("confidence", 0.0)),
                        float(count) / 4.0),
                )
                confirmed.append(confirmed_candidate)
        self.room_entry_observations[floor_index] = current
        return confirmed

    def _room_progress_stalled(self, pose):
        """True when the active room task has made no pose progress for too long.

        Compares the current pose against the last point that registered
        meaningful motion (translation or heading).  A genuine halt -- neither
        the body nor the heading advancing for ``room_entry_stall_timeout`` --
        is treated as a stall so the caller can abandon the room instead of
        wedging the robot in place.  The progress marker is refreshed while the
        robot is still advancing, so only a real stop starts the clock.
        """
        now = rospy.Time.now()
        if (self.room_entry_progress_time == rospy.Time(0) or
                self.room_entry_progress_pose is None):
            self.room_entry_progress_pose = pose
            self.room_entry_progress_time = now
            return False
        last = self.room_entry_progress_pose
        moved = math.hypot(pose[0] - last[0], pose[1] - last[1])
        turned = abs(((pose[3] - last[3] + math.pi) % (2 * math.pi)) - math.pi)
        if moved >= 0.25 or turned >= 0.25:
            self.room_entry_progress_pose = pose
            self.room_entry_progress_time = now
            return False
        return now - self.room_entry_progress_time > rospy.Duration(
            self.room_entry_stall_timeout
        )

    def _update_floor_graph(self, belief, floor_index):
        """Record observed corridor anchors and side-room entry nodes."""
        if floor_index >= len(self.floor_graphs):
            return
        graph = self.floor_graphs[floor_index]
        root_id = self._graph_root(floor_index)
        candidates = self._confirmed_room_entry_candidates(
            self._room_entry_candidates(belief), floor_index
        )
        for candidate in candidates:
            # A corridor position is shared by both side rooms.  Keeping one
            # anchor per longitudinal position makes the corridor a chain,
            # instead of a collection of unrelated root-to-room spokes.
            key = "%.2f" % candidate["y"]
            anchor_id = self.graph_anchor_nodes[floor_index].get(key)
            if anchor_id is None:
                for existing_id in self.graph_anchor_nodes[floor_index].values():
                    existing = graph.nodes[existing_id]
                    if abs(existing.y - candidate["y"]) <= 0.75:
                        anchor_id = existing_id
                        break
            if anchor_id is None:
                anchor_id, _ = graph.add_node(
                    "junction",
                    floor_index,
                    0.0,
                    candidate["y"],
                    corridor_s=candidate["corridor_s"],
                    confidence=candidate["confidence"],
                    label="corridor_anchor_%s" % key,
                )
                self.graph_anchor_nodes[floor_index][key] = anchor_id
            else:
                self.graph_anchor_nodes[floor_index][key] = anchor_id
            entry_id, _ = graph.add_room_entry(
                floor_index,
                candidate["x"],
                candidate["y"],
                yaw=candidate["yaw"],
                corridor_s=candidate["corridor_s"],
                confidence=candidate["confidence"],
                label="room_entry_%s" % key,
            )
            graph.add_edge(anchor_id, entry_id)

        corridor_nodes = [
            root_id,
        ]
        corridor_nodes.extend(
            node.node_id for node in sorted(
                graph.nodes.values(),
                key=lambda item: (
                    float("inf") if item.corridor_s is None else item.corridor_s,
                    item.node_id,
                ),
            )
            if node.node_type == "junction" and node.floor == floor_index
        )
        end_id = self.graph_end_nodes[floor_index]
        if end_id is not None:
            corridor_nodes.append(end_id)
        for start_id, end_id in zip(corridor_nodes, corridor_nodes[1:]):
            graph.add_edge(start_id, end_id)
        self._refresh_room_door_ys(floor_index)

    def _refresh_room_door_ys(self, floor_index):
        """Derive door y coordinates from lidar-observed room_entry nodes."""
        if floor_index >= len(self.floor_graphs):
            return []
        graph = self.floor_graphs[floor_index]
        observed_ys = sorted({
            float(node.y)
            for node in graph.nodes.values()
            if (node.node_type == "room_entry" and
                node.floor == floor_index and
                math.isfinite(float(node.y)))
        })
        door_ys = []
        for observed_y in observed_ys:
            if not door_ys or abs(observed_y - door_ys[-1]) > 0.75:
                door_ys.append(observed_y)
        self.room_door_ys_by_floor[floor_index] = door_ys
        if floor_index == getattr(self, "current_floor", floor_index):
            self.room_door_ys = list(door_ys)
        return door_ys

    def _prepare_graph_dfs(self, floor_index):
        """Build a stable DFS event order from the discovered corridor end."""
        if floor_index >= len(self.floor_graphs):
            return False
        end_id = self.graph_end_nodes[floor_index]
        if end_id is None:
            return False
        graph = self.floor_graphs[floor_index]
        order = graph.dfs_order(end_id, floor=floor_index)
        if not order:
            return False
        self.graph_dfs_order[floor_index] = order
        self.graph_dfs_index[floor_index] = 0
        self.graph_dfs_attempts[floor_index] = 0
        self.graph_phase[floor_index] = "dfs"
        self.status = "dfs_started floor=%d nodes=%d" % (
            floor_index, len(order)
        )
        return True

    def _plan_graph_corridor_survey(self, belief, pose, floor_index):
        """Drive to the corridor end before selecting room tasks."""
        if floor_index >= len(self.graph_phase) or not self.corridor_bounds:
            return False
        if self.graph_phase[floor_index] != "corridor_discovery":
            return False
        corridor = self.corridor_bounds
        corridor_min_y = float(corridor["y_min"])
        if pose[1] < corridor_min_y - 0.5:
            entrance_goal = (0.0, corridor_min_y + 0.8)
            path = self._astar_path(
                belief,
                (pose[0], pose[1]),
                entrance_goal,
                allow_unknown=True,
                restrict_roi=False,
            )
            if path:
                self._set_path(path, "graph_main_entrance")
                return True
        end_y = float(corridor["y_max"]) - 0.8
        if pose[1] >= end_y - 0.5:
            graph = self.floor_graphs[floor_index]
            end_id, _ = graph.add_node(
                "corridor_end",
                floor_index,
                0.0,
                end_y,
                corridor_s=end_y - corridor_min_y,
                confidence=1.0,
                label="corridor_end",
            )
            self.graph_end_nodes[floor_index] = end_id
            self._update_floor_graph(belief, floor_index)
            room_count = sum(
                1 for node in graph.nodes.values()
                if node.node_type == "room_entry"
            )
            self._prepare_graph_dfs(floor_index)
            self.status = "corridor_end_discovered rooms=%d dfs_nodes=%d" % (
                room_count, len(self.graph_dfs_order[floor_index])
            )
            self.path = []
            self.path_index = 0
            return True
        path = self._astar_path(
            belief,
            (pose[0], pose[1]),
            (0.0, end_y),
            allow_unknown=False,
            restrict_roi=True,
        )
        if not path:
            # The corridor may still contain unobserved cells immediately
            # after the entrance.  Unknown-space recovery remains bounded by
            # the building ROI and never crosses occupied cells.
            path = self._astar_path(
                belief,
                (pose[0], pose[1]),
                (0.0, end_y),
                allow_unknown=True,
                restrict_roi=True,
            )
        if not path:
            return False
        self._set_path(path, "graph_corridor_survey")
        return True

    def _room_side(self, node):
        corridor = self.corridor_bounds or {}
        center_x = 0.5 * (
            float(corridor.get("x_min", 0.0)) +
            float(corridor.get("x_max", 0.0))
        )
        return "left" if node.x < center_x else "right"

    def _room_bounds(self, node):
        """Infer a room rectangle from neighboring observed entry nodes."""
        corridor = self.corridor_bounds
        if not corridor:
            return None
        y_min = float(corridor["y_min"])
        y_max = float(corridor["y_max"])
        graph = self.floor_graphs[node.floor]
        side = self._room_side(node)
        y_values = sorted({
            round(other.y, 2) for other in graph.nodes.values()
            if (other.node_type == "room_entry" and
                other.floor == node.floor and self._room_side(other) == side)
        })
        lower = y_min
        upper = y_max
        for value in y_values:
            if value < node.y - 0.4:
                lower = max(lower, value)
            elif value > node.y + 0.4:
                upper = min(upper, value)
                break
        footprint = self.footprint_bounds or {}
        if side == "left":
            x_min = float(footprint.get("x_min", corridor["x_min"] - 8.0))
            x_max = float(corridor["x_min"])
        else:
            x_min = float(corridor["x_max"])
            x_max = float(footprint.get("x_max", corridor["x_max"] + 8.0))
        if x_max <= x_min or upper <= lower:
            return None
        return {
            "x_min": x_min,
            "x_max": x_max,
            "y_min": lower,
            "y_max": upper,
        }

    def _room_mask(self, bounds):
        x_values = self.map_origin_x + np.arange(self.map_cells) * self.cell_size
        y_values = self.map_origin_y + np.arange(self.map_cells) * self.cell_size
        return (
            (x_values[None, :] >= float(bounds["x_min"])) &
            (x_values[None, :] <= float(bounds["x_max"])) &
            (y_values[:, None] >= float(bounds["y_min"])) &
            (y_values[:, None] <= float(bounds["y_max"]))
        )

    def _room_target(self, node):
        bounds = self._room_bounds(node)
        if bounds is None:
            return None
        return (
            0.5 * (bounds["x_min"] + bounds["x_max"]),
            0.5 * (bounds["y_min"] + bounds["y_max"]),
        )

    def _room_coverage(self, belief, node):
        bounds = self._room_bounds(node)
        if bounds is None:
            return 0.0
        return self._coverage_in_world_bounds(belief, bounds)

    @staticmethod
    def _frontier_clusters(frontier_mask):
        """Return 8-connected frontier components as lists of (x, y) cells."""
        height, width = frontier_mask.shape
        visited = np.zeros_like(frontier_mask, dtype=bool)
        clusters = []
        for seed_y, seed_x in np.argwhere(frontier_mask):
            if visited[seed_y, seed_x]:
                continue
            queue = deque([(int(seed_x), int(seed_y))])
            visited[seed_y, seed_x] = True
            cluster = []
            while queue:
                cell_x, cell_y = queue.popleft()
                cluster.append((cell_x, cell_y))
                for dx, dy in (
                        (-1, -1), (0, -1), (1, -1),
                        (-1, 0),           (1, 0),
                        (-1, 1),  (0, 1),  (1, 1)):
                    next_x, next_y = cell_x + dx, cell_y + dy
                    if (0 <= next_x < width and 0 <= next_y < height and
                            frontier_mask[next_y, next_x] and
                            not visited[next_y, next_x]):
                        visited[next_y, next_x] = True
                        queue.append((next_x, next_y))
            clusters.append(cluster)
        return clusters

    @staticmethod
    def _distance_tree(traversable, start):
        """Build one BFS tree reused by all room candidates."""
        height, width = traversable.shape
        distance = np.full((height, width), -1, dtype=np.int32)
        parent_x = np.full((height, width), -1, dtype=np.int16)
        parent_y = np.full((height, width), -1, dtype=np.int16)
        if start is None:
            return distance, parent_x, parent_y
        start_x, start_y = start
        if not (0 <= start_x < width and 0 <= start_y < height):
            return distance, parent_x, parent_y
        queue = deque([start])
        distance[start_y, start_x] = 0
        while queue:
            cell_x, cell_y = queue.popleft()
            next_distance = distance[cell_y, cell_x] + 1
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                next_x, next_y = cell_x + dx, cell_y + dy
                if (not (0 <= next_x < width and 0 <= next_y < height) or
                        not traversable[next_y, next_x] or
                        distance[next_y, next_x] >= 0):
                    continue
                distance[next_y, next_x] = next_distance
                parent_x[next_y, next_x] = cell_x
                parent_y[next_y, next_x] = cell_y
                queue.append((next_x, next_y))
        return distance, parent_x, parent_y

    @staticmethod
    def _sample_cluster_cells(cluster, count):
        """Choose spatially separated representatives from one frontier cluster."""
        if not cluster or count <= 0:
            return []
        center_x = sum(cell[0] for cell in cluster) / float(len(cluster))
        center_y = sum(cell[1] for cell in cluster) / float(len(cluster))
        first = min(
            cluster,
            key=lambda cell: ((cell[0] - center_x) ** 2 +
                              (cell[1] - center_y) ** 2),
        )
        selected = [first]
        remaining = set(cluster)
        remaining.discard(first)
        while remaining and len(selected) < count:
            next_cell = max(
                remaining,
                key=lambda cell: min(
                    (cell[0] - chosen[0]) ** 2 +
                    (cell[1] - chosen[1]) ** 2
                    for chosen in selected
                ),
            )
            selected.append(next_cell)
            remaining.remove(next_cell)
        return selected

    def _room_information_gain(self, belief, room_mask, candidate):
        """Count unique unknown cells visible from a candidate by 2-D ray casting."""
        max_cells = max(
            1, int(math.ceil(self.room_information_range / self.cell_size))
        )
        ray_count = max(
            8,
            int(math.ceil(360.0 / self.room_information_angle_step_deg)),
        )
        visible_unknown = set()
        cell_x, cell_y = candidate
        for ray_index in range(ray_count):
            angle = 2.0 * math.pi * ray_index / float(ray_count)
            end_x = cell_x + int(round(max_cells * math.cos(angle)))
            end_y = cell_y + int(round(max_cells * math.sin(angle)))
            for ray_x, ray_y in bresenham(cell_x, cell_y, end_x, end_y):
                if not self._inside(ray_x, ray_y) or not room_mask[ray_y, ray_x]:
                    break
                value = belief[ray_y, ray_x]
                if value == OCCUPIED:
                    break
                if value == UNKNOWN:
                    visible_unknown.add((ray_x, ray_y))
        return len(visible_unknown)

    def _blacklist_reached_room_target(self, pose):
        """Avoid selecting the same observation point after its path completes."""
        target = self.active_room_target_cell
        if target is None:
            return
        target_world = (
            self.map_origin_x + target[0] * self.cell_size,
            self.map_origin_y + target[1] * self.cell_size,
        )
        if math.hypot(pose[0] - target_world[0], pose[1] - target_world[1]) > 0.90:
            return
        radius = int(math.ceil(
            self.room_target_blacklist_radius / self.cell_size
        ))
        for offset_y in range(-radius, radius + 1):
            for offset_x in range(-radius, radius + 1):
                if offset_x * offset_x + offset_y * offset_y > radius * radius:
                    continue
                self.active_room_frontier_blacklist.add(
                    (target[0] + offset_x, target[1] + offset_y)
                )
        self.active_room_target_cell = None

    def _room_frontier_path(self, belief, pose, node):
        """Select a clustered room frontier using gain, travel and return cost."""
        bounds = self._room_bounds(node)
        if bounds is None:
            return []
        room_mask = self._room_mask(bounds)
        frontier = self._frontier_mask(belief) & room_mask
        for cell_x, cell_y in self.active_room_frontier_blacklist:
            if self._inside(cell_x, cell_y):
                frontier[cell_y, cell_x] = False

        traversable = self._known_free_mask(belief) & room_mask
        start = self._cell((pose[0], pose[1]))
        if start is None:
            return []
        traversable[start[1], start[0]] = True
        distance, parent_x, parent_y = self._distance_tree(traversable, start)

        door_cell = self._cell(node.position)
        if door_cell is not None:
            traversable[door_cell[1], door_cell[0]] = True
        door_distance, _, _ = self._distance_tree(traversable, door_cell)

        clusters = [
            cluster for cluster in self._frontier_clusters(frontier)
            if len(cluster) >= self.room_frontier_min_cluster_cells
        ]
        clusters.sort(key=len, reverse=True)
        candidates = []
        for cluster in clusters:
            reachable_cluster = [
                cell for cell in cluster
                if distance[cell[1], cell[0]] >= 2
            ]
            candidates.extend(self._sample_cluster_cells(
                reachable_cluster, self.room_candidates_per_cluster
            ))
            if len(candidates) >= self.room_candidate_limit:
                break
        candidates = candidates[:self.room_candidate_limit]
        if not candidates:
            self.last_room_plan = {
                "candidate_count": 0,
                "cluster_count": len(clusters),
            }
            return []

        best_cell = None
        best_score = -float("inf")
        best_record = None
        for cell_x, cell_y in candidates:
            path_cells = int(distance[cell_y, cell_x])
            if path_cells < 2:
                continue
            gain_cells = self._room_information_gain(
                belief, room_mask, (cell_x, cell_y)
            )
            gain_area = gain_cells * self.cell_size * self.cell_size
            path_m = path_cells * self.cell_size
            return_cells = int(door_distance[cell_y, cell_x])
            if return_cells >= 0:
                return_m = return_cells * self.cell_size
            elif door_cell is not None:
                return_m = math.hypot(
                    cell_x - door_cell[0], cell_y - door_cell[1]
                ) * self.cell_size
            else:
                return_m = 0.0
            obstacle_penalty = (
                self.room_obstacle_cost_weight
                if self._near_obstacle(belief, cell_x, cell_y, 2)
                else 0.0
            )
            score = (
                gain_area -
                self.room_path_cost_weight * path_m -
                self.room_return_cost_weight * return_m -
                obstacle_penalty
            )
            if score > best_score:
                best_score = score
                best_cell = (int(cell_x), int(cell_y))
                best_record = {
                    "cell": best_cell,
                    "score": float(score),
                    "gain_cells": int(gain_cells),
                    "gain_area": float(gain_area),
                    "path_m": float(path_m),
                    "return_m": float(return_m),
                    "candidate_count": len(candidates),
                    "cluster_count": len(clusters),
                }
        if best_cell is None:
            return []

        cells = []
        current = best_cell
        while current is not None:
            cells.append(current)
            if current == start:
                break
            previous = (
                int(parent_x[current[1], current[0]]),
                int(parent_y[current[1], current[0]]),
            )
            current = None if min(previous) < 0 else previous
        if not cells or cells[-1] != start:
            return []
        cells.reverse()
        self.active_room_target_cell = best_cell
        self.last_room_plan = best_record
        return self._cells_to_waypoints(cells)

    @staticmethod
    def _append_path(result, path):
        for point in path:
            if (not result or
                    math.hypot(point[0] - result[-1][0],
                               point[1] - result[-1][1]) > 0.10):
                result.append(point)

    def _room_path(self, belief, pose, node, entering):
        """Plan through the doorway, or back to the corridor, for one room."""
        path = []
        corridor_pose = (0.0, node.y)
        entry_pose = node.position
        if entering:
            approach = self._astar_path(
                belief,
                (pose[0], pose[1]),
                corridor_pose,
                allow_unknown=False,
                restrict_roi=True,
            )
            if not approach and math.hypot(
                    pose[0] - corridor_pose[0], pose[1] - corridor_pose[1]) > 0.55:
                return []
            self._append_path(path, approach)
            doorway = self._astar_path(
                belief,
                corridor_pose,
                entry_pose,
                allow_unknown=True,
                restrict_roi=True,
            )
            if not doorway:
                return []
            self._append_path(path, doorway)
            target = self._room_target(node)
            if target is None:
                return []
            interior = self._astar_path(
                belief,
                entry_pose,
                target,
                allow_unknown=True,
                restrict_roi=True,
            )
            if not interior:
                return []
            self._append_path(path, interior)
            return path

        interior_exit = self._astar_path(
            belief,
            (pose[0], pose[1]),
            entry_pose,
            allow_unknown=False,
            restrict_roi=True,
        )
        if not interior_exit:
            interior_exit = self._astar_path(
                belief,
                (pose[0], pose[1]),
                entry_pose,
                allow_unknown=True,
                restrict_roi=True,
            )
        if (not interior_exit and
                math.hypot(pose[0] - entry_pose[0],
                           pose[1] - entry_pose[1]) <= 0.90):
            # A pose already at the observed doorway can share its grid cell
            # with the entry node, so no A* segment is needed here.
            interior_exit = [entry_pose]
        if not interior_exit:
            return []
        self._append_path(path, interior_exit)
        corridor_exit = self._astar_path(
            belief,
            entry_pose,
            corridor_pose,
            allow_unknown=False,
            restrict_roi=True,
        )
        if not corridor_exit:
            # The doorway can remain partly unknown after interior scanning;
            # use the observed entry geometry as a controlled recovery route.
            corridor_exit = self._astar_path(
                belief,
                entry_pose,
                corridor_pose,
                allow_unknown=True,
                restrict_roi=True,
            )
        if (not corridor_exit and
                math.hypot(pose[0] - entry_pose[0],
                           pose[1] - entry_pose[1]) <= 0.90):
            # A door node is created only after a free opening is observed.
            # If inflation still seals that opening, cross it geometrically
            # from the entry neighborhood rather than routing through a wall.
            corridor_exit = [entry_pose, corridor_pose]
        if not corridor_exit:
            return []
        self._append_path(path, corridor_exit)
        return path

    def _pose_inside_room(self, pose, node):
        bounds = self._room_bounds(node)
        if bounds is None:
            return False
        margin = 0.45
        return (
            float(bounds["x_min"]) + margin <= pose[0] <=
            float(bounds["x_max"]) - margin and
            float(bounds["y_min"]) + margin <= pose[1] <=
            float(bounds["y_max"]) - margin
        )

    def _plan_graph_room_task(self, belief, pose, floor_index):
        """Execute the current floor's DFS events one at a time.

        Corridor nodes are motion events.  Room-entry nodes are delegated to
        the room state machine below.  The order starts at the far corridor
        end, so a straight corridor is processed from far to near while a
        branch is naturally handled by DFS backtracking.
        """
        if (floor_index >= len(self.graph_phase) or
                self.graph_phase[floor_index] not in ("dfs", "room_reverse")):
            return False
        graph = self.floor_graphs[floor_index]

        # Keep the old phase name recoverable for a state left by a process
        # restart, but all new runs use the explicit DFS event list.
        if self.graph_phase[floor_index] == "room_reverse":
            if not self._prepare_graph_dfs(floor_index):
                self.graph_phase[floor_index] = "frontier_fallback"
                self.status = "room_graph_unavailable"
                return False

        if self.active_room_node is None:
            order = self.graph_dfs_order[floor_index]
            known_nodes = set(order)
            if any(
                    node.node_type == "room_entry" and
                    node.node_id not in known_nodes
                    for node in graph.nodes.values()):
                # A late scan may reveal an entrance after the first survey.
                # Rebuild the event list so the new room is not silently
                # omitted from DFS -- but preserve our current DFS position, so
                # a late doorway does not reset the walk back to the corridor
                # end and trap the robot in a two-point oscillation.
                resume_node = None
                idx = self.graph_dfs_index[floor_index]
                if 0 <= idx < len(order):
                    resume_node = order[idx]
                if not self._prepare_graph_dfs(floor_index):
                    self.graph_phase[floor_index] = "frontier_fallback"
                    self.status = "room_graph_unavailable"
                    return False
                order = self.graph_dfs_order[floor_index]
                if resume_node is not None:
                    try:
                        self.graph_dfs_index[floor_index] = order.index(resume_node)
                    except ValueError:
                        self.graph_dfs_index[floor_index] = 0

            while self.graph_dfs_index[floor_index] < len(order):
                node_id = order[self.graph_dfs_index[floor_index]]
                node = graph.nodes[node_id]
                if node.node_type == "room_entry":
                    if node.completed or node.blocked:
                        self.graph_dfs_index[floor_index] += 1
                        continue
                    self.active_room_node = node_id
                    self.active_room_phase = "entering"
                    self.active_room_entered = False
                    self.active_room_started = rospy.Time.now()
                    self.active_room_no_frontier_cycles = 0
                    self.active_room_target_cell = None
                    self.active_room_frontier_blacklist = set()
                    self.room_entry_path_failures = 0
                    self.last_room_plan = None
                    self.room_entry_progress_pose = pose
                    self.room_entry_progress_time = rospy.Time.now()
                    self.graph_dfs_attempts[floor_index] = 0
                    graph.mark_node(node_id, visited=True)
                    self.path = []
                    self.path_index = 0
                    self.status = "dfs_room_selected id=%d y=%.2f" % (
                        node.node_id, node.y
                    )
                    break

                if node.node_type not in (
                        "entrance", "junction", "corridor_end"):
                    self.graph_dfs_index[floor_index] += 1
                    continue

                distance = math.hypot(node.x - pose[0], node.y - pose[1])
                if distance <= 0.65:
                    graph.mark_node(node_id, visited=True)
                    self.graph_dfs_index[floor_index] += 1
                    self.graph_dfs_attempts[floor_index] = 0
                    self.path = []
                    self.path_index = 0
                    continue
                if self.path_index < len(self.path):
                    return True
                path = self._astar_path(
                    belief,
                    (pose[0], pose[1]),
                    node.position,
                    allow_unknown=False,
                    restrict_roi=True,
                )
                if path:
                    self._set_path(
                        path, "dfs_corridor_segment id=%d" % node.node_id
                    )
                    return True
                self.graph_dfs_attempts[floor_index] += 1
                attempts = self.graph_dfs_attempts[floor_index]
                if attempts < 3:
                    self.status = "dfs_waiting_for_segment id=%d attempt=%d" % (
                        node.node_id, attempts
                    )
                    self.path = []
                    self.path_index = 0
                    return True
                graph.mark_node(node_id, blocked=True)
                self.graph_dfs_index[floor_index] += 1
                self.graph_dfs_attempts[floor_index] = 0
                self.status = "dfs_segment_blocked id=%d" % node.node_id

            if self.active_room_node is None:
                self.graph_phase[floor_index] = "frontier_fallback"
                self.status = "room_graph_complete"
                self.path = []
                self.path_index = 0
                return False

        node = graph.nodes[self.active_room_node]
        elapsed = (rospy.Time.now() - self.active_room_started).to_sec()
        if self.active_room_phase == "entering":
            target = self._room_target(node)
            if (self._pose_inside_room(pose, node) or
                    (target is not None and math.hypot(
                        pose[0] - target[0], pose[1] - target[1]
                    ) <= 0.65)):
                self.active_room_entered = True
                node.entered = True
                self.active_room_phase = "surveying"
                self.path = []
                self.path_index = 0
                self.status = "room_entered id=%d" % node.node_id
                return True
            if self.path_index < len(self.path):
                if self._room_progress_stalled(pose):
                    graph.mark_node(self.active_room_node, blocked=True)
                    self.status = "room_blocked_stalled id=%d" % node.node_id
                    self.active_room_node = None
                    self.active_room_phase = None
                    self.room_entry_progress_pose = None
                    self.room_entry_progress_time = rospy.Time(0)
                    self.graph_dfs_index[floor_index] += 1
                    return True
                return True
            path = self._room_path(belief, pose, node, entering=True)
            if path:
                self.room_entry_path_failures = 0
                self._set_path(path, "room_enter_path id=%d" % node.node_id)
                return True
            self.room_entry_path_failures += 1
            if (elapsed >= self.room_task_timeout or
                    self.room_entry_path_failures >= self.room_entry_path_fail_limit):
                graph.mark_node(self.active_room_node, blocked=True)
                self.status = "room_blocked id=%d entry_failures=%d" % (
                    node.node_id, self.room_entry_path_failures
                )
                self.active_room_node = None
                self.active_room_phase = None
                self.graph_dfs_index[floor_index] += 1
                return True
            self.status = "room_waiting_for_entry_path id=%d failures=%d" % (
                node.node_id, self.room_entry_path_failures
            )
            self.path = []
            self.path_index = 0
            return True

        if self.active_room_phase == "surveying":
            coverage = self._room_coverage(belief, node)
            node.coverage = max(node.coverage, coverage)
            if self.active_room_entered and coverage > 0.05:
                node.interior_observation = True
            if (self.active_room_entered and
                    coverage >= self.min_room_coverage):
                self.active_room_phase = "returning"
                self.status = "room_coverage_reached id=%d %.3f" % (
                    node.node_id, coverage
                )
            else:
                if self.path_index < len(self.path):
                    if self._room_progress_stalled(pose):
                        graph.mark_node(self.active_room_node, blocked=True)
                        self.status = "room_blocked_frontier_stalled id=%d" % node.node_id
                        self.active_room_node = None
                        self.active_room_phase = None
                        self.room_entry_progress_pose = None
                        self.room_entry_progress_time = rospy.Time(0)
                        self.graph_dfs_index[floor_index] += 1
                        return True
                    return True
                self._blacklist_reached_room_target(pose)
                path = self._room_frontier_path(belief, pose, node)
                if path:
                    self.active_room_no_frontier_cycles = 0
                    plan = self.last_room_plan or {}
                    self._set_path(
                        path,
                        ("room_frontier id=%d coverage=%.3f gain=%s "
                         "path=%.2f return=%.2f score=%.2f") % (
                            node.node_id,
                            coverage,
                            plan.get("gain_cells", "?"),
                            plan.get("path_m", -1.0),
                            plan.get("return_m", -1.0),
                            plan.get("score", -1.0),
                        ),
                    )
                    return True
                self.active_room_no_frontier_cycles += 1
                self.status = "room_waiting_for_frontier id=%d %.3f" % (
                    node.node_id, coverage
                )
                self.path = []
                self.path_index = 0
                if elapsed < self.room_task_timeout:
                    return True
                graph.mark_node(self.active_room_node, blocked=True)
                self.status = "room_blocked_no_frontier id=%d" % node.node_id
                self.active_room_node = None
                self.active_room_phase = None
                self.graph_dfs_index[floor_index] += 1
                return True

        if self.active_room_phase == "returning":
            corridor = self.corridor_bounds or {}
            in_corridor = (
                float(corridor.get("x_min", -1.1)) - 0.35 <= pose[0] <=
                float(corridor.get("x_max", 1.1)) + 0.35 and
                float(corridor.get("y_min", -float("inf"))) - 0.35 <= pose[1] <=
                float(corridor.get("y_max", float("inf"))) + 0.35
            )
            if in_corridor:
                graph.mark_node(self.active_room_node, completed=True)
                node.coverage = max(node.coverage, self._room_coverage(belief, node))
                self.status = "room_complete id=%d" % node.node_id
                self.active_room_node = None
                self.active_room_phase = None
                self.graph_dfs_index[floor_index] += 1
                self.graph_dfs_attempts[floor_index] = 0
                self.path = []
                self.path_index = 0
                return True
            if self.path_index < len(self.path):
                if self._room_progress_stalled(pose):
                    graph.mark_node(self.active_room_node, blocked=True)
                    self.status = "room_return_blocked_stalled id=%d" % node.node_id
                    self.active_room_node = None
                    self.active_room_phase = None
                    self.room_entry_progress_pose = None
                    self.room_entry_progress_time = rospy.Time(0)
                    self.graph_dfs_index[floor_index] += 1
                    return True
                return True
            path = self._room_path(belief, pose, node, entering=False)
            if path:
                self._set_path(path, "room_return_path id=%d" % node.node_id)
                return True
            if elapsed >= self.room_task_timeout:
                graph.mark_node(self.active_room_node, blocked=True)
                self.status = "room_return_blocked id=%d" % node.node_id
                self.active_room_node = None
                self.active_room_phase = None
                self.graph_dfs_index[floor_index] += 1
                return True
            self.status = "room_waiting_for_return_path id=%d" % node.node_id
            self.path = []
            self.path_index = 0
            return True
        return False

    def _safe_stair_entry_path(self, pose):
        """Use known door openings before entering the shared corridor."""
        corridor = self.corridor_bounds
        entry = self._stair_entry("up")
        if not corridor or entry is None:
            return []
        x_min = float(corridor["x_min"])
        x_max = float(corridor["x_max"])
        y_min = float(corridor["y_min"])
        y_max = float(corridor["y_max"])
        corridor_x = min(max(0.0, x_min + 0.45), x_max - 0.45)
        if pose[1] <= y_min:
            return [entry]
        door_ys = self._refresh_room_door_ys(
            getattr(self, "current_floor", 0)
        )
        if not door_ys:
            # Without a lidar-observed room entry, let the regular A* fallback
            # use the current occupancy map instead of inventing a door pose.
            return []
        door_y = min(door_ys, key=lambda value: abs(value - pose[1]))
        path = []
        if pose[0] > x_max + 0.25:
            route_x = pose[0]
            if (abs(pose[1] - door_y) >=
                    self._waypoint_tolerance("go_stairs_up")):
                room_approach_x = x_max + 0.80
            else:
                room_approach_x = pose[0]
            if abs(pose[0] - room_approach_x) > 0.45:
                path.append((room_approach_x, pose[1]))
                route_x = room_approach_x
            if abs(pose[1] - door_y) < self._waypoint_tolerance("go_stairs_up"):
                offset = 0.45 if pose[1] <= door_y else -0.45
                path.append((route_x, door_y + offset))
            path.extend(((route_x, door_y), (x_max - 0.45, door_y)))
        elif pose[0] < x_min - 0.25:
            route_x = pose[0]
            if (abs(pose[1] - door_y) >=
                    self._waypoint_tolerance("go_stairs_up")):
                room_approach_x = x_min - 0.80
            else:
                room_approach_x = pose[0]
            if abs(pose[0] - room_approach_x) > 0.45:
                path.append((room_approach_x, pose[1]))
                route_x = room_approach_x
            if abs(pose[1] - door_y) < self._waypoint_tolerance("go_stairs_up"):
                offset = 0.45 if pose[1] <= door_y else -0.45
                path.append((route_x, door_y + offset))
            path.extend(((route_x, door_y), (x_min + 0.45, door_y)))
        else:
            path.append((
                min(max(pose[0], x_min + 0.45), x_max - 0.45),
                min(max(pose[1], y_min + 0.45), y_max - 0.45),
            ))
        path.extend(((corridor_x, y_min - 0.45), entry))
        return path

    def _build_exploration_roi(self):
        roi = np.ones((self.map_cells, self.map_cells), dtype=bool)
        if self.footprint_bounds is not None:
            roi.fill(False)
            margin = 0.35
            x_min = self.footprint_bounds["x_min"] - margin
            x_max = self.footprint_bounds["x_max"] + margin
            y_min = self.footprint_bounds["y_min"] - margin
            y_max = self.footprint_bounds["y_max"] + margin
            entrance_x = 0.5 * (
                self.footprint_bounds["x_min"] + self.footprint_bounds["x_max"]
            )
            corridor_x_min = min(entrance_x, self.home[0]) - 1.25
            corridor_x_max = max(entrance_x, self.home[0]) + 1.25
            corridor_y_min = min(self.home[1], self.footprint_bounds["y_min"]) - 0.5
            corridor_y_max = max(self.home[1], self.footprint_bounds["y_min"]) + 0.5
            for cell_y in range(self.map_cells):
                world_y = self.map_origin_y + cell_y * self.cell_size
                for cell_x in range(self.map_cells):
                    world_x = self.map_origin_x + cell_x * self.cell_size
                    inside_building = (
                        x_min <= world_x <= x_max and y_min <= world_y <= y_max
                    )
                    inside_entrance_corridor = (
                        corridor_x_min <= world_x <= corridor_x_max and
                        corridor_y_min <= world_y <= corridor_y_max
                    )
                    if inside_building or inside_entrance_corridor:
                        roi[cell_y, cell_x] = True
        core_bounds = []
        if self.stair_bounds is not None:
            core_bounds.append(self.stair_bounds)
        elevator_bounds = getattr(self, "elevator_bounds", None)
        if elevator_bounds:
            core_bounds.append(elevator_bounds)
        lobby_bounds = getattr(self, "lobby_bounds", None)
        margin = 0.20
        for bounds in core_bounds:
            y_bounds = lobby_bounds or bounds
            x_min = float(bounds["x_min"]) - margin
            x_max = float(bounds["x_max"]) + margin
            y_min = float(y_bounds["y_min"]) - margin
            y_max = float(y_bounds["y_max"]) + margin
            for cell_y in range(self.map_cells):
                world_y = self.map_origin_y + cell_y * self.cell_size
                for cell_x in range(self.map_cells):
                    world_x = self.map_origin_x + cell_x * self.cell_size
                    if x_min <= world_x <= x_max and y_min <= world_y <= y_max:
                        roi[cell_y, cell_x] = False
        self.exploration_roi = roi

    def _floor_from_height(self, z_value):
        if self.home is None or self.floor_height <= 0.0:
            return 0
        relative_floor = (float(z_value) - self.home[2]) / self.floor_height
        return int(clamp(round(relative_floor), 0, self.floor_count - 1))

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        if not all(math.isfinite(value) for value in (position.x, position.y, position.z,
                                                       orientation.x, orientation.y,
                                                       orientation.z, orientation.w)):
            rospy.logerr_throttle(5.0, "Odometry contains NaN/Inf")
            return
        with self.lock:
            new_pose = (float(position.x), float(position.y), float(position.z),
                        float(yaw_from_quaternion(orientation)))
            previous_floor = self.current_floor
            self.pose = new_pose
            if self.home is None:
                self.home = self.pose
                self.map_origin_x = self.home[0] - self.map_size_m / 2.0
                self.map_origin_y = self.home[1] - self.map_size_m / 2.0
                self._build_exploration_roi()
                self.last_progress_pose = self.pose
                self.last_progress_time = rospy.Time.now()
                rospy.loginfo("home pose recorded at (%.3f, %.3f)", self.home[0], self.home[1])
            self.current_floor = self._floor_from_height(position.z)
            if self.current_floor != previous_floor:
                rospy.loginfo(
                    "floor transition observed: %d -> %d at z=%.3f",
                    previous_floor,
                    self.current_floor,
                    position.z,
                )
            if self.last_distance_pose is not None:
                last_floor, last_x, last_y = self.last_distance_pose
                step = math.hypot(new_pose[0] - last_x, new_pose[1] - last_y)
                if last_floor == self.current_floor and step < 1.0:
                    self.floor_distance[self.current_floor] += step
            self.last_distance_pose = (
                self.current_floor, new_pose[0], new_pose[1]
            )

    def scan_callback(self, msg):
        with self.lock:
            if self.pose is None or self.map_origin_x is None:
                return
            pose = self.pose
            floor_index = self.current_floor
            origin_x, origin_y = self.map_origin_x, self.map_origin_y
            if self.mode in ("stairs_up", "stairs_down"):
                self.latest_scan_stamp = msg.header.stamp
                self.last_map_update = rospy.Time.now()
                return

        points = list(msg.points)
        if not points:
            return
        stride = max(self.scan_stride, int(math.ceil(float(len(points)) / self.max_scan_points)))
        sampled = points[::stride]
        transformed = self._transform_points(msg.header.frame_id, msg.header.stamp, sampled, pose)
        if transformed is None:
            return

        robot_cell = self._cell((pose[0], pose[1]), origin_x, origin_y)
        sensor_origin = self._sensor_origin(msg.header.frame_id, msg.header.stamp, pose)
        if robot_cell is None or sensor_origin is None:
            return
        sensor_cell = self._cell((sensor_origin[0], sensor_origin[1]), origin_x, origin_y)
        if sensor_cell is None:
            sensor_cell = robot_cell

        with self.lock:
            belief = self.floor_beliefs[floor_index]
            obs_hits = self.obs_hits[floor_index]
            obs_free = self.obs_free[floor_index]
            new_known_cells = 0
            for point in transformed:
                px, py, pz = point
                if not all(math.isfinite(value) for value in (px, py, pz)):
                    continue
                distance = math.hypot(px - pose[0], py - pose[1])
                if (distance > self.scan_max_range or
                        distance < self.scan_min_range):
                    continue
                if (self.scan_self_filter_enabled and
                        self._is_self_return((px, py), pose)):
                    continue
                # Keep the current floor and reject high ceiling returns.
                if pz < pose[2] - 0.05 or pz > pose[2] + 1.0:
                    continue
                endpoint = self._cell((px, py), origin_x, origin_y)
                if endpoint is None:
                    continue
                line = list(bresenham(sensor_cell[0], sensor_cell[1], endpoint[0], endpoint[1]))
                for idx, (cell_x, cell_y) in enumerate(line[:-1]):
                    if not self._inside(cell_x, cell_y):
                        continue
                    if belief[cell_y, cell_x] == OCCUPIED:
                        # A free ray now crosses an occupied cell.  Count it as
                        # a free pass only when the ray re-enters FREE space
                        # before its endpoint (_free_past): a phantom cell or
                        # cluster left by 3-D flattening / odometry drift floats
                        # in open space, whereas a genuine wall is opaque and
                        # never lets a ray reach FREE beyond it.  This keeps
                        # drift-displaced wall cells from being eroded into
                        # phantom doorways while still clearing real ghosts.
                        if self._free_past(belief, line, idx):
                            free = obs_free[cell_y, cell_x] + 1
                            obs_free[cell_y, cell_x] = free
                            total = free + obs_hits[cell_y, cell_x]
                            if (total >= self.obstacle_clear_min_obs and
                                    free * 100.0 / total >
                                    self.obstacle_clear_percent):
                                belief[cell_y, cell_x] = FREE
                                obs_free[cell_y, cell_x] = 0
                                obs_hits[cell_y, cell_x] = 0
                    else:
                        if (belief[cell_y, cell_x] == UNKNOWN and
                                (self.exploration_roi is None or
                                 self.exploration_roi[cell_y, cell_x])):
                            new_known_cells += 1
                        belief[cell_y, cell_x] = FREE
                        obs_free[cell_y, cell_x] = 0
                        obs_hits[cell_y, cell_x] = 0
                ex, ey = endpoint
                if self._inside(ex, ey):
                    if (belief[ey, ex] == UNKNOWN and
                            (self.exploration_roi is None or
                             self.exploration_roi[ey, ex])):
                        new_known_cells += 1
                    if belief[ey, ex] == OCCUPIED:
                        obs_hits[ey, ex] += 1
                    else:
                        obs_hits[ey, ex] = 1
                        obs_free[ey, ex] = 0
                    belief[ey, ex] = OCCUPIED
            if self._inside(robot_cell[0], robot_cell[1]):
                if (belief[robot_cell[1], robot_cell[0]] == UNKNOWN and
                        (self.exploration_roi is None or
                         self.exploration_roi[robot_cell[1], robot_cell[0]])):
                    new_known_cells += 1
                belief[robot_cell[1], robot_cell[0]] = FREE
                obs_free[robot_cell[1], robot_cell[0]] = 0
                obs_hits[robot_cell[1], robot_cell[0]] = 0
            self.pending_known_cells[floor_index] += new_known_cells
            if (self.pending_known_cells[floor_index] >=
                    self.map_progress_cell_batch):
                self.last_progress_pose = pose
                self.last_progress_time = rospy.Time.now()
                self.pending_known_cells[floor_index] = 0
            self.latest_scan_stamp = msg.header.stamp
            self.last_map_update = rospy.Time.now()

    def _free_past(self, belief, line, idx):
        """True if the ray re-enters FREE space after the occupied cell at
        line[idx] and before it terminates at the (occupied) endpoint.

        A floating phantom cell/cluster (3-D flattening or odometry drift) sits
        in open space, so a ray that passes through it keeps going through FREE
        cells.  A genuine wall is opaque: the cells directly behind it are
        UNKNOWN (occluded) and the endpoint itself is OCCUPIED, so a ray never
        reaches FREE beyond a real wall.  Requiring FREE beyond before counting
        a free-pass therefore erases ghosts while leaving wall cells intact
        (no phantom doorways from eroding a real wall)."""
        for j in range(idx + 1, len(line)):
            cx, cy = line[j]
            if not self._inside(cx, cy):
                return False
            v = belief[cy, cx]
            if v == FREE:
                return True
            if v != OCCUPIED:
                # UNKNOWN (occluded) -> solid wall face.
                return False
        return False

    def _is_self_return(self, point, pose):
        """Reject returns inside the robot footprint in the base frame."""
        dx = float(point[0]) - float(pose[0])
        dy = float(point[1]) - float(pose[1])
        cos_yaw = math.cos(float(pose[3]))
        sin_yaw = math.sin(float(pose[3]))
        forward = cos_yaw * dx + sin_yaw * dy
        lateral = -sin_yaw * dx + cos_yaw * dy
        return (
            -self.scan_self_filter_rear <= forward <= self.scan_self_filter_front
            and -self.scan_self_filter_right <= lateral <= self.scan_self_filter_left
        )

    def _transform_points(self, source_frame, stamp, points, pose):
        if not source_frame or source_frame in (self.map_frame, "odom", "/odom"):
            return [(float(point.x), float(point.y), float(point.z)) for point in points]
        try:
            translation, rotation = self._lookup_sensor_transform(source_frame, stamp)
            matrix = transformations.quaternion_matrix(rotation)
            matrix[0:3, 3] = translation
            values = np.ones((len(points), 4), dtype=np.float64)
            values[:, 0] = [point.x for point in points]
            values[:, 1] = [point.y for point in points]
            values[:, 2] = [point.z for point in points]
            values = np.dot(matrix, values.T).T
            return values[:, 0:3].tolist()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(5.0, "TF unavailable: %s -> %s", source_frame, self.map_frame)
            return None

    def _sensor_origin(self, source_frame, stamp, pose):
        if not source_frame or source_frame in (self.map_frame, "odom", "/odom"):
            return pose[0], pose[1], pose[2]
        try:
            translation, _ = self._lookup_sensor_transform(source_frame, stamp)
            return float(translation[0]), float(translation[1]), float(translation[2])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            if source_frame.strip("/") == "laser_livox":
                return (pose[0] + 0.2 * math.cos(pose[3]),
                        pose[1] + 0.2 * math.sin(pose[3]),
                        pose[2] + 0.08)
            return None

    def _lookup_sensor_transform(self, source_frame, stamp):
        query_stamp = stamp if stamp != rospy.Time(0) else rospy.Time(0)
        try:
            return self.tf_listener.lookupTransform(
                self.map_frame, source_frame, query_stamp
            )
        except tf.ExtrapolationException:
            return self.tf_listener.lookupTransform(
                self.map_frame, source_frame, rospy.Time(0)
            )

    def _cell(self, point, origin_x=None, origin_y=None):
        origin_x = self.map_origin_x if origin_x is None else origin_x
        origin_y = self.map_origin_y if origin_y is None else origin_y
        x = int(round((point[0] - origin_x) / self.cell_size))
        y = int(round((point[1] - origin_y) / self.cell_size))
        return (x, y) if self._inside(x, y) else None

    def _inside(self, x, y):
        return 0 <= x < self.map_cells and 0 <= y < self.map_cells

    def start_callback(self, _request):
        with self.lock:
            if self.home is None:
                return TriggerResponse(False, "waiting for /Odometry_gazebo")
            self.active = True
            self.mode = "explore"
            self.path = []
            self.path_index = 0
            self.current_frontier_cell = None
            self.frontier_progress_cell = None
            self.frontier_best_distance = float("inf")
            self.frontier_progress_time = rospy.Time.now()
            self.started_at = rospy.Time.now()
            self.full_map_complete = False
            self._reset_graph_state()
            self.return_after_transition = False
            self.no_frontier_cycles = [0 for _ in range(self.floor_count)]
            self.distributed_coverage_cycles = [0 for _ in range(self.floor_count)]
            self.status = "explore_requested"
        rospy.loginfo("exploration started")
        return TriggerResponse(True, "exploration started")

    def return_home_callback(self, _request):
        with self.lock:
            if self.home is None:
                return TriggerResponse(False, "home pose is not recorded")
            self.active = True
            self.return_after_transition = True
            self.mode = "go_stairs_down" if self.current_floor > self.home_floor else "return"
            self.transition_target_floor = (
                self.current_floor - 1 if self.current_floor > self.home_floor else None
            )
            self.path = []
            self.path_index = 0
            self.status = "return_requested"
        rospy.loginfo("return-home requested")
        return TriggerResponse(True, "return-home requested")

    def set_home_callback(self, _request):
        with self.lock:
            if self.pose is None:
                return TriggerResponse(False, "waiting for odometry")
            self.home = self.pose
            self.status = "home_updated"
        rospy.loginfo("home pose updated to (%.3f, %.3f)", self.home[0], self.home[1])
        return TriggerResponse(True, "home pose updated")

    def stop_callback(self, _request):
        with self.lock:
            self.active = False
            self.mode = "idle"
            self.path = []
            self.path_index = 0
            self.status = "stopped"
        self.publish_zero()
        return TriggerResponse(True, "navigation stopped")

    def _coverage_ratio(self, belief):
        roi = self.exploration_roi
        if roi is None:
            roi = np.ones_like(belief, dtype=bool)
        denominator = int(np.count_nonzero(roi))
        if denominator == 0:
            return 0.0
        return float(np.count_nonzero((belief != UNKNOWN) & roi)) / denominator

    def _coverage_in_world_bounds(self, belief, bounds):
        if self.map_origin_x is None or self.map_origin_y is None:
            return 0.0
        x0 = max(
            0,
            int(math.floor(
                (float(bounds["x_min"]) - self.map_origin_x) / self.cell_size
            )),
        )
        x1 = min(
            self.map_cells,
            int(math.ceil(
                (float(bounds["x_max"]) - self.map_origin_x) / self.cell_size
            )) + 1,
        )
        y0 = max(
            0,
            int(math.floor(
                (float(bounds["y_min"]) - self.map_origin_y) / self.cell_size
            )),
        )
        y1 = min(
            self.map_cells,
            int(math.ceil(
                (float(bounds["y_max"]) - self.map_origin_y) / self.cell_size
            )) + 1,
        )
        if x1 <= x0 or y1 <= y0:
            return 0.0
        region = belief[y0:y1, x0:x1]
        return float(np.count_nonzero(region != UNKNOWN)) / int(region.size)

    def _distributed_coverage_complete(self, belief, coverage, floor_index):
        if not self._graph_floor_ready(floor_index):
            return False
        if (coverage < self.completion_min_floor_coverage or
                self.floor_distance[floor_index] <
                self.completion_min_floor_distance or
                self.footprint_bounds is None):
            return False
        x_mid = 0.5 * (
            self.footprint_bounds["x_min"] + self.footprint_bounds["x_max"]
        )
        y_min = self.footprint_bounds["y_min"]
        y_step = (
            self.footprint_bounds["y_max"] - y_min
        ) / 3.0
        sectors = []
        for row in range(3):
            sector_y_min = y_min + row * y_step
            sector_y_max = y_min + (row + 1) * y_step
            sectors.extend((
                {
                    "x_min": self.footprint_bounds["x_min"],
                    "x_max": x_mid,
                    "y_min": sector_y_min,
                    "y_max": sector_y_max,
                },
                {
                    "x_min": x_mid,
                    "x_max": self.footprint_bounds["x_max"],
                    "y_min": sector_y_min,
                    "y_max": sector_y_max,
                },
            ))
        return all(
            self._coverage_in_world_bounds(belief, bounds) >=
            self.completion_min_sector_coverage
            for bounds in sectors
        )

    def _graph_floor_ready(self, floor_index):
        """Require every discovered room on a floor to finish before exit.

        A room that was discovered but could not be entered (blocked) counts as
        satisfied: a drift-induced phantom doorway is detected as a room, then
        blocked after entry fails, and must not hold the whole floor open for
        re-exploration forever.  Only genuinely-missing rooms (fewer than
        ``expected``) keep the floor from completing."""
        if floor_index >= len(self.floor_graphs):
            return False
        if self.active_room_node is not None:
            return False
        rooms = [
            node for node in self.floor_graphs[floor_index].nodes.values()
            if node.node_type == "room_entry"
        ]
        expected = (
            self.expected_room_count[floor_index]
            if floor_index < len(self.expected_room_count) else 0
        )
        if expected and len(rooms) < expected:
            return False
        return bool(rooms) and all(
            node.completed or node.blocked for node in rooms
        )

    def _room_task_status(self):
        records = []
        for floor_index, graph in enumerate(self.floor_graphs):
            for node in sorted(graph.nodes.values(), key=lambda item: item.node_id):
                if node.node_type != "room_entry":
                    continue
                side = "L" if node.x < 0.0 else "R"
                records.append(
                    "%d:%d:%s:%.2f:%d:%d:%d:%d:%.3f" % (
                        floor_index,
                        node.node_id,
                        side,
                        node.y,
                        int(node.entered),
                        int(node.interior_observation),
                        int(node.completed),
                        int(node.blocked),
                        node.coverage,
                    )
                )
        return ",".join(records) if records else "none"

    def _visited_neighborhood_mask(self, floor_index):
        """Boolean mask of cells within frontier_revisit_radius of any visited cell."""
        mask = np.zeros((self.map_cells, self.map_cells), dtype=bool)
        radius = self.frontier_revisit_radius_cells
        if radius <= 0:
            return mask
        for cell_x, cell_y in self.visited_cells[floor_index]:
            x0 = max(0, cell_x - radius)
            x1 = min(self.map_cells, cell_x + radius + 1)
            y0 = max(0, cell_y - radius)
            y1 = min(self.map_cells, cell_y + radius + 1)
            mask[y0:y1, x0:x1] = True
        return mask

    def _reachable_frontier_plan(self, belief, pose, floor_index):
        """Return a high-information reachable frontier and its known-free path."""
        frontier = self._exploration_frontier_mask(belief)
        traversable = self._known_free_mask(belief)
        if self.exploration_roi is not None:
            traversable &= self.exploration_roi
        for cell_x, cell_y in self.blacklisted_frontiers[floor_index]:
            if self._inside(cell_x, cell_y):
                frontier[cell_y, cell_x] = False

        start = self._cell((pose[0], pose[1]))
        if start is None:
            return [], None, 0
        traversable[start[1], start[0]] = True
        distance = np.full(traversable.shape, -1, dtype=np.int32)
        parent_x = np.full(traversable.shape, -1, dtype=np.int16)
        parent_y = np.full(traversable.shape, -1, dtype=np.int16)
        queue = deque([start])
        distance[start[1], start[0]] = 0
        while queue:
            cell_x, cell_y = queue.popleft()
            next_distance = distance[cell_y, cell_x] + 1
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cell_x + dx, cell_y + dy
                if (not self._inside(nx, ny) or not traversable[ny, nx] or
                        distance[ny, nx] >= 0):
                    continue
                distance[ny, nx] = next_distance
                parent_x[ny, nx] = cell_x
                parent_y[ny, nx] = cell_y
                queue.append((nx, ny))

        candidates = np.argwhere(frontier & (distance >= 2))
        if candidates.size == 0:
            return [], None, 0
        unknown = belief == UNKNOWN
        best_cell = None
        current_frontier_cell = getattr(self, "current_frontier_cell", None)
        if current_frontier_cell is not None:
            current_x, current_y = current_frontier_cell
            if (self._inside(current_x, current_y) and
                    frontier[current_y, current_x] and
                    distance[current_y, current_x] >= 2):
                best_cell = (current_x, current_y)
        if best_cell is None:
            best_score = -float("inf")
            visited_near = self._visited_neighborhood_mask(floor_index)
            for cell_y, cell_x in candidates:
                if visited_near[cell_y, cell_x]:
                    continue
                radius = 5
                y0 = max(0, cell_y - radius)
                y1 = min(self.map_cells, cell_y + radius + 1)
                x0 = max(0, cell_x - radius)
                x1 = min(self.map_cells, cell_x + radius + 1)
                information_gain = int(np.count_nonzero(unknown[y0:y1, x0:x1]))
                score = information_gain + 0.4 * float(distance[cell_y, cell_x])
                if score > best_score:
                    best_score = score
                    best_cell = (int(cell_x), int(cell_y))

        cells = []
        current = best_cell
        while current is not None:
            cells.append(current)
            if current == start:
                break
            px = int(parent_x[current[1], current[0]])
            py = int(parent_y[current[1], current[0]])
            current = None if px < 0 or py < 0 else (px, py)
        if not cells or cells[-1] != start:
            return [], None, len(candidates)
        cells.reverse()
        target = (
            self.map_origin_x + best_cell[0] * self.cell_size,
            self.map_origin_y + best_cell[1] * self.cell_size,
        )
        return self._cells_to_waypoints(cells), target, len(candidates)

    def _stair_geometry(self):
        bounds = self.stair_bounds
        if not bounds:
            return None
        width = float(bounds["x_max"]) - float(bounds["x_min"])
        x_left = float(bounds["x_min"]) + 0.26 * width
        x_right = float(bounds["x_max"]) - 0.26 * width
        x_exit = float(bounds["x_max"]) + 0.5 * width
        y_entry = float(bounds["y_min"]) + 0.65
        y_turn = float(bounds["y_max"]) - 1.65
        return x_left, x_right, y_entry, y_turn, x_exit

    @staticmethod
    def _waypoint_tolerance(mode):
        if mode in ("stairs_up", "stairs_down"):
            return 0.25
        return 0.35

    @staticmethod
    def _yaw_rate_limit(mode, max_yaw_rate, stair_yaw_rate=1.0):
        if mode in ("stairs_up", "stairs_down"):
            return min(max_yaw_rate, stair_yaw_rate)
        return max_yaw_rate

    @staticmethod
    def _stair_speed_cap(
        mode,
        path_index,
        stair_speed,
        landing_speed,
        max_linear,
        flat_use_max_linear=True,
    ):
        """Return the linear speed cap for a stair route waypoint segment."""
        if mode not in ("stairs_up", "stairs_down"):
            return max_linear
        if path_index in (1, 3):
            return stair_speed
        if path_index == 2:
            return landing_speed
        if path_index in (0, 4) and not flat_use_max_linear:
            return stair_speed
        return max_linear

    @staticmethod
    def _tracking_waypoint(mode, path_index, pose, waypoint, lookahead=0.30):
        if mode not in ("stairs_up", "stairs_down") or path_index not in (1, 3):
            return waypoint
        delta_y = waypoint[1] - pose[1]
        # Keep the moving target within roughly one physical tread.  A long
        # preview skips several risers and can wedge the base against an edge.
        if abs(delta_y) <= lookahead:
            return waypoint
        return (waypoint[0], pose[1] + math.copysign(lookahead, delta_y))

    @staticmethod
    def _stair_cruise_command(
        mode,
        path_index,
        body_x,
        body_y,
        heading_error,
        waypoint_distance,
        stair_speed,
        brake_distance,
        heading_tolerance=0.60,
        lateral_tolerance=0.18,
    ):
        """Return a cruise command for a well-aligned stair flight."""
        if mode not in ("stairs_up", "stairs_down") or path_index not in (1, 3):
            return None
        if waypoint_distance <= brake_distance:
            return None
        if body_x <= 0.0:
            return None
        if abs(body_y) > lateral_tolerance:
            return None
        if abs(heading_error) > heading_tolerance:
            return None
        return math.copysign(abs(stair_speed), body_x)

    def _stair_entry(self, direction):
        geometry = self._stair_geometry()
        if geometry is None:
            return None
        _, _, y_entry, _, x_exit = geometry
        return (x_exit, y_entry)

    def _stair_route(self, direction):
        geometry = self._stair_geometry()
        if geometry is None:
            return []
        x_left, x_right, y_entry, y_turn, x_exit = geometry
        if direction == "up":
            return [
                (x_left, y_entry),
                (x_left, y_turn),
                (x_right, y_turn),
                (x_right, y_entry),
                (x_exit, y_entry),
            ]
        return [
            (x_right, y_entry),
            (x_right, y_turn),
            (x_left, y_turn),
            (x_left, y_entry),
            (x_exit, y_entry),
        ]

    def _begin_stair_transition(self, direction):
        route = self._stair_route(direction)
        if not route:
            self.status = "stair_geometry_unavailable"
            return False
        self.mode = "stairs_%s" % direction
        self.path = route
        self.path_index = 0
        self.status = "stairs_%s_started target_floor=%d" % (
            direction, self.transition_target_floor
        )
        self.current_frontier_cell = None
        rospy.loginfo(
            "starting stair transition %s: floor %d -> %d",
            direction,
            self.current_floor,
            self.transition_target_floor,
        )
        return True

    def _finish_stair_transition(self):
        target_floor = self.transition_target_floor
        if target_floor is None or self.current_floor != target_floor:
            return False
        route = self.path
        if route and self.pose is not None:
            finish_tolerance = max(
                self._waypoint_tolerance(self.mode), 0.40
            )
            if (math.hypot(self.pose[0] - route[-1][0],
                           self.pose[1] - route[-1][1]) >
                    finish_tolerance):
                return False
        rospy.loginfo("stair transition reached floor %d", self.current_floor)
        self.path = []
        self.path_index = 0
        self.no_frontier_cycles[self.current_floor] = 0
        self.distributed_coverage_cycles[self.current_floor] = 0
        if self.return_after_transition:
            if self.current_floor > self.home_floor:
                self.transition_target_floor = self.current_floor - 1
                self.mode = "go_stairs_down"
                self.status = "continue_return_via_stairs"
            else:
                self.transition_target_floor = None
                self.mode = "return"
                self.status = "return_on_home_floor"
        else:
            self.transition_target_floor = None
            self.mode = "explore"
            self.status = "explore_floor_%d" % self.current_floor
        return True

    def _mark_floor_complete(self, floor_index):
        self.floor_complete[floor_index] = True
        rospy.loginfo(
            "floor %d exploration complete: coverage=%.3f distance=%.2f",
            floor_index,
            self.floor_coverage[floor_index],
            self.floor_distance[floor_index],
        )
        if floor_index < self.floor_count - 1:
            self.transition_target_floor = floor_index + 1
            self.return_after_transition = False
            self.mode = "go_stairs_up"
            self.path = []
            self.path_index = 0
            self.status = "floor_%d_complete_go_stairs" % floor_index
            return
        self.full_map_complete = all(self.floor_complete)
        self.return_after_transition = True
        if self.current_floor > self.home_floor:
            self.transition_target_floor = self.current_floor - 1
            self.mode = "go_stairs_down"
            self.status = "full_map_complete_return_via_stairs"
        else:
            self.mode = "return"
            self.status = "full_map_complete_return"
        self.path = []
        self.path_index = 0

    def _plan_to_stairs(self, belief, pose, direction):
        entry = self._stair_entry(direction)
        if entry is None:
            self.status = "stair_geometry_unavailable"
            return
        if math.hypot(pose[0] - entry[0], pose[1] - entry[1]) <= 0.55:
            self._begin_stair_transition(direction)
            return
        safe_path = self._safe_stair_entry_path(pose)
        if safe_path:
            self._set_path(safe_path, "go_stairs_%s_door_route" % direction)
            return
        path = self._astar_path(
            belief,
            (pose[0], pose[1]),
            entry,
            allow_unknown=False,
            restrict_roi=True,
        )
        if not path:
            self.status = "stair_entry_path_unavailable_%s" % direction
            rospy.logwarn_throttle(5.0, "known-free path to stair entry is unavailable")
            return
        self._set_path(path, "go_stairs_%s_planned" % direction)

    def planning_timer(self, _event):
        with self.lock:
            if self.pose is None or not self.active:
                return
            if self.auto_return_after_sec > 0.0 and self.started_at is not None:
                elapsed = (rospy.Time.now() - self.started_at).to_sec()
                if elapsed >= self.auto_return_after_sec and self.mode == "explore":
                    self.return_after_transition = True
                    if self.current_floor > self.home_floor:
                        self.transition_target_floor = self.current_floor - 1
                        self.mode = "go_stairs_down"
                    else:
                        self.mode = "return"
                    self.path = []
                    self.path_index = 0
                    self.status = "automatic_return_timeout"
            pose = self.pose
            mode = self.mode
            floor_index = self.current_floor
            belief = self.floor_beliefs[floor_index].copy()
            home = self.home
            map_ready = self.last_map_update != rospy.Time(0)

        try:
            coverage = self._coverage_ratio(belief)
            with self.lock:
                self.floor_coverage[floor_index] = coverage
            if mode != "return" and not map_ready:
                with self.lock:
                    self.status = "waiting_for_first_scan"
                rospy.logwarn_throttle(5.0, "waiting for the first sensor map before planning")
                return
            self._update_floor_graph(belief, floor_index)
            if mode in ("stairs_up", "stairs_down"):
                with self.lock:
                    self._finish_stair_transition()
                return
            if mode in ("go_stairs_up", "go_stairs_down"):
                direction = "up" if mode.endswith("up") else "down"
                with self.lock:
                    self._plan_to_stairs(belief, pose, direction)
                return
            if mode == "return":
                if home is None:
                    return
                if floor_index > self.home_floor:
                    with self.lock:
                        self.return_after_transition = True
                        self.transition_target_floor = floor_index - 1
                        self.mode = "go_stairs_down"
                        self.path = []
                        self.path_index = 0
                        self.status = "return_requires_floor_transition"
                    return
                goal = (home[0], home[1])
                new_path = self._astar_path(belief, (pose[0], pose[1]), goal,
                                            allow_unknown=self.allow_unknown_return)
                if not new_path:
                    with self.lock:
                        self.status = "return_path_unavailable"
                    rospy.logwarn_throttle(5.0, "return path unavailable; holding zero velocity")
                    return
                self._set_path(new_path, "return_planned")
                return

            if mode != "explore":
                return

            if self._plan_graph_corridor_survey(belief, pose, floor_index):
                return
            if self._plan_graph_room_task(belief, pose, floor_index):
                return

            now = rospy.Time.now()
            if self._distributed_coverage_complete(belief, coverage, floor_index):
                with self.lock:
                    self.distributed_coverage_cycles[floor_index] += 1
                    cycles = self.distributed_coverage_cycles[floor_index]
                    self.status = "survey_distributed_coverage cycles=%d" % cycles
                    if cycles >= self.completion_cycles_required:
                        self._mark_floor_complete(floor_index)
                return
            with self.lock:
                self.distributed_coverage_cycles[floor_index] = 0

            if (self.current_frontier_cell is not None and
                    self.last_progress_time != rospy.Time(0) and
                    now - self.last_progress_time > rospy.Duration(30.0) and
                    self.frontier_progress_time != rospy.Time(0) and
                    now - self.frontier_progress_time > rospy.Duration(10.0)):
                frontier_x, frontier_y = self.current_frontier_cell
                with self.lock:
                    for dx in range(-2, 3):
                        for dy in range(-2, 3):
                            self.blacklisted_frontiers[floor_index].add(
                                (frontier_x + dx, frontier_y + dy)
                            )
                    self.current_frontier_cell = None
                    self.frontier_progress_cell = None
                    self.frontier_best_distance = float("inf")
                    self.frontier_progress_time = now
                    self.path = []
                    self.path_index = 0
                    self.pending_known_cells[floor_index] = 0
                    self.last_progress_pose = pose
                    self.last_progress_time = now
                    self.status = "stalled_frontier_blacklisted"
                rospy.logwarn("stalled exploration frontier blacklisted on floor %d", floor_index)

            frontier_path, target, reachable_count = self._reachable_frontier_plan(
                belief, pose, floor_index
            )
            with self.lock:
                self.reachable_frontiers[floor_index] = reachable_count
            if not frontier_path or target is None:
                with self.lock:
                    self.current_frontier_cell = None
                    self.frontier_progress_cell = None
                    self.frontier_best_distance = float("inf")
                    if (self._graph_floor_ready(floor_index) and
                            coverage >= self.min_floor_coverage):
                        self.no_frontier_cycles[floor_index] += 1
                        cycles = self.no_frontier_cycles[floor_index]
                        self.status = "survey_no_frontier cycles=%d" % cycles
                        if cycles >= self.no_frontier_cycles_required:
                            self._mark_floor_complete(floor_index)
                    else:
                        self.no_frontier_cycles[floor_index] = 0
                        self.status = "survey_coverage_incomplete %.3f<%.3f" % (
                            coverage, self.min_floor_coverage
                        )
                return

            target_cell = self._cell(target)
            target_distance = math.hypot(target[0] - pose[0], target[1] - pose[1])
            with self.lock:
                self.no_frontier_cycles[floor_index] = 0
                self.current_frontier_cell = target_cell
                if target_cell != self.frontier_progress_cell:
                    self.frontier_progress_cell = target_cell
                    self.frontier_best_distance = target_distance
                    self.frontier_progress_time = now
                elif target_distance <= self.frontier_best_distance - 0.40:
                    self.frontier_best_distance = target_distance
                    self.frontier_progress_time = now

            if not self.policy.ready:
                if frontier_path:
                    self._set_path(frontier_path, "frontier_astar_fallback")
                    return

            (nodes, utility, visited, current_index, target_index,
             neighbors, centers, adjacency) = self._build_graph(
                belief, pose, target, floor_index, frontier_path
            )
            if (len(nodes) < 2 or not neighbors or not centers or
                    target_index == current_index):
                self._set_path(frontier_path, "hdplanner_graph_recovery")
                return
            selected_index = self.policy.select(
                nodes, utility, visited, current_index, target_index,
                neighbors, centers, adjacency
            )
            if selected_index is None or selected_index == current_index:
                self._set_path(frontier_path, "hdplanner_action_recovery")
                return
            else:
                plan_status = "hdplanner_policy_selected"
            goal = nodes[selected_index]
            current_to_target = float(np.linalg.norm(nodes[current_index] - target))
            selected_to_target = float(np.linalg.norm(goal - target))
            if selected_to_target > current_to_target - 0.20:
                self._set_path(
                    frontier_path,
                    "hdplanner_nonprogress_recovery inference=%d" %
                    self.policy.inference_count,
                )
                return
            new_path = self._astar_path(
                belief,
                (pose[0], pose[1]),
                (goal[0], goal[1]),
                allow_unknown=False,
                restrict_roi=True,
            )
            if not new_path:
                self._set_path(frontier_path, "hdplanner_selected_path_recovery")
                return
            self._set_path(
                new_path,
                "%s inference=%d center=%d logp=%.5f" % (
                    plan_status,
                    self.policy.inference_count,
                    self.policy.last_selected_center,
                    self.policy.last_action_logp,
                ),
            )
        except Exception as exc:
            rospy.logerr_throttle(5.0, "planning exception: %s\n%s", exc, traceback.format_exc())
            with self.lock:
                self.status = "planning_exception"
                self.path = []
                self.path_index = 0

    def _frontier_path(self, belief, pose):
        frontier = self._exploration_frontier_mask(belief)
        cells = np.argwhere(frontier)
        if cells.size == 0:
            return []
        start = np.array([pose[0], pose[1]], dtype=np.float32)
        candidates = []
        for cell_y, cell_x in cells:
            point = np.array([
                self.map_origin_x + cell_x * self.cell_size,
                self.map_origin_y + cell_y * self.cell_size,
            ], dtype=np.float32)
            distance = float(np.linalg.norm(point - start))
            if 1.0 <= distance <= 18.0:
                candidates.append((distance, (float(point[0]), float(point[1]))))
        candidates.sort(key=lambda item: item[0], reverse=True)
        best_path = []
        for _, candidate in candidates[:40]:
            path = self._astar_path(belief, (pose[0], pose[1]), candidate, allow_unknown=False)
            if len(path) > len(best_path):
                best_path = path
        return best_path

    def _build_graph(self, belief, pose, target, floor_index, frontier_path):
        spacing = 4.0
        origin_x, origin_y = self.map_origin_x, self.map_origin_y
        center_x = origin_x + round((pose[0] - origin_x) / spacing) * spacing
        center_y = origin_y + round((pose[1] - origin_y) / spacing) * spacing
        current = np.array([pose[0], pose[1]], dtype=np.float32)
        nodes = [current]
        traversable = self._known_free_mask(belief)
        if self.exploration_roi is not None:
            traversable &= self.exploration_roi
        for ix in range(-7, 8):
            for iy in range(-7, 8):
                node = np.array([center_x + ix * spacing, center_y + iy * spacing], dtype=np.float32)
                cell = self._cell(node, origin_x, origin_y)
                if cell is None or not traversable[cell[1], cell[0]]:
                    continue
                if np.linalg.norm(node - current) < 0.8:
                    continue
                if all(np.linalg.norm(node - existing) > 0.2 for existing in nodes):
                    nodes.append(node)
        current_index = 0
        guide_node = None
        for waypoint in frontier_path[1:]:
            candidate = np.array(waypoint, dtype=np.float32)
            candidate_distance = float(np.linalg.norm(candidate - current))
            if candidate_distance > 3.5:
                break
            if (candidate_distance >= 0.8 and
                    self._line_is_free(
                        traversable, current, candidate, origin_x, origin_y
                    )):
                guide_node = candidate
        if (guide_node is not None and
                all(np.linalg.norm(guide_node - existing) > 0.2 for existing in nodes)):
            nodes.append(guide_node)
        target_node = np.array(target, dtype=np.float32)
        target_index = min(
            range(len(nodes)),
            key=lambda index: float(np.linalg.norm(nodes[index] - target_node)),
        )
        if np.linalg.norm(nodes[target_index] - target_node) > 0.2:
            nodes.append(target_node)
            target_index = len(nodes) - 1
        utility = []
        visited = []
        frontier = self._exploration_frontier_mask(belief)
        frontier_cells = np.argwhere(frontier)
        frontier_points = np.empty((len(frontier_cells), 2), dtype=np.float32)
        if len(frontier_cells):
            frontier_points[:, 0] = origin_x + frontier_cells[:, 1] * self.cell_size
            frontier_points[:, 1] = origin_y + frontier_cells[:, 0] * self.cell_size
        for node in nodes:
            key = (round(float(node[0]), 1), round(float(node[1]), 1))
            was_visited = any(
                math.hypot(key[0] - old[0], key[1] - old[1]) <= 2.0
                for old in self.visited_by_floor[floor_index]
            )
            visited.append(1 if was_visited else 0)
            utility.append(0 if was_visited else self._node_utility(
                traversable, node, frontier_points, origin_x, origin_y
            ))

        adjacency = []
        for index, node in enumerate(nodes):
            adjacent = []
            for other_index, other in enumerate(nodes):
                if other_index == index:
                    continue
                delta = np.abs(other - node)
                if (delta[0] <= 2.01 * spacing and delta[1] <= 2.01 * spacing and
                        self._line_is_free(traversable, node, other, origin_x, origin_y)):
                    adjacent.append(other_index)
            adjacent.sort(key=lambda other_index: float(np.linalg.norm(nodes[other_index] - node)))
            adjacency.append(adjacent[:self.policy.K_SIZE - 1])
        neighbors = adjacency[current_index]
        candidates = [index for index, score in enumerate(utility) if score > 0 and index != current_index]
        centers = self._sparsify_centers(
            traversable, nodes, candidates, target_index, origin_x, origin_y
        )
        if target_index != current_index and target_index not in centers:
            centers.append(target_index)
        centers = centers[:self.policy.K_SIZE]
        for center in centers:
            if center != target_index:
                if center not in adjacency[target_index]:
                    adjacency[target_index].append(center)
                if target_index not in adjacency[center]:
                    adjacency[center].append(target_index)
        return (nodes, utility, visited, current_index, target_index,
                neighbors, centers, adjacency)

    def _node_utility(self, traversable, node, frontier_points, origin_x, origin_y):
        if not len(frontier_points):
            return 0
        distances = np.linalg.norm(frontier_points - node, axis=1)
        visible = 0
        for point in frontier_points[distances < 4.0]:
            if self._line_is_free(traversable, node, point, origin_x, origin_y):
                visible += 1
                if visible > 2:
                    return 1
        return 0

    def _line_is_free(self, traversable, start, end, origin_x, origin_y):
        start_cell = self._cell(start, origin_x, origin_y)
        end_cell = self._cell(end, origin_x, origin_y)
        if start_cell is None or end_cell is None:
            return False
        for cell_x, cell_y in bresenham(
                start_cell[0], start_cell[1], end_cell[0], end_cell[1]):
            if not self._inside(cell_x, cell_y) or not traversable[cell_y, cell_x]:
                return False
        return True

    def _sparsify_centers(self, traversable, nodes, candidates, target_index,
                          origin_x, origin_y):
        if not candidates:
            return []
        target = nodes[target_index]
        ordered = sorted(
            candidates,
            key=lambda index: float(np.linalg.norm(nodes[index] - target)),
        )
        selected = []
        for index in ordered:
            covered = False
            for center in selected:
                if (np.linalg.norm(nodes[index] - nodes[center]) <= 20.0 and
                        self._line_is_free(
                            traversable, nodes[index], nodes[center], origin_x, origin_y
                        )):
                    covered = True
                    break
            if not covered:
                selected.append(index)
            if len(selected) >= self.policy.K_SIZE:
                break
        return selected

    @staticmethod
    def _dilate_mask(mask, radius):
        result = np.zeros_like(mask, dtype=bool)
        height, width = mask.shape
        for dy in range(-radius, radius + 1):
            source_y = slice(max(0, -dy), min(height, height - dy))
            target_y = slice(max(0, dy), min(height, height + dy))
            for dx in range(-radius, radius + 1):
                source_x = slice(max(0, -dx), min(width, width - dx))
                target_x = slice(max(0, dx), min(width, width + dx))
                result[target_y, target_x] |= mask[source_y, source_x]
        return result

    def _known_free_mask(self, belief):
        free = belief == FREE
        occupied = belief == OCCUPIED
        # Inflation radii are expressed in metres so the physical clearance the
        # planner keeps from obstacles is independent of the grid resolution.
        # At the original 0.4 m cells these reproduce the previous hardcoded
        # counts: free dilated 2 cells (0.8 m), obstacles inflated 1 cell
        # (0.4 m).  Raising the resolution (smaller cell_size) therefore keeps
        # the same clearance instead of shrinking it.
        free_radius = max(2, int(math.ceil(0.8 / self.cell_size)))
        obstacle_radius = max(1, int(math.ceil(0.4 / self.cell_size)))
        observed_free = self._dilate_mask(free, free_radius)
        blocked = self._dilate_mask(occupied, obstacle_radius)
        traversable = observed_free & ~blocked

        occupied_up = np.zeros_like(occupied, dtype=bool)
        occupied_down = np.zeros_like(occupied, dtype=bool)
        occupied_left = np.zeros_like(occupied, dtype=bool)
        occupied_right = np.zeros_like(occupied, dtype=bool)
        doorway_offsets = sorted({
            max(1, int(round(0.4 / self.cell_size))),
            max(1, int(round(0.8 / self.cell_size))),
        })
        for offset in doorway_offsets:
            occupied_up[offset:, :] |= occupied[:-offset, :]
            occupied_down[:-offset, :] |= occupied[offset:, :]
            occupied_left[:, offset:] |= occupied[:, :-offset]
            occupied_right[:, :-offset] |= occupied[:, offset:]
        doorway_seed = free & (
            (occupied_up & occupied_down) |
            (occupied_left & occupied_right)
        )
        doorway_radius = max(2, int(math.ceil(0.8 / self.cell_size)))
        doorway_clearance = self._dilate_mask(doorway_seed, doorway_radius)
        traversable |= doorway_clearance & free

        # Door panels leave too little clearance after global obstacle inflation.
        # Restore only cells the sensor has positively observed as free at the
        # configured main entrance; unknown and occupied cells remain blocked.
        footprint_bounds = getattr(self, "footprint_bounds", None)
        if footprint_bounds is not None:
            entrance = (
                0.5 * (
                    footprint_bounds["x_min"] +
                    footprint_bounds["x_max"]
                ),
                footprint_bounds["y_min"],
            )
            entrance_cell = self._cell(entrance)
            if entrance_cell is not None:
                radius_x = max(1, int(math.ceil(0.45 / self.cell_size)))
                radius_y = max(1, int(math.ceil(0.80 / self.cell_size)))
                cell_x, cell_y = entrance_cell
                x0 = max(0, cell_x - radius_x)
                x1 = min(self.map_cells, cell_x + radius_x + 1)
                y0 = max(0, cell_y - radius_y)
                y1 = min(self.map_cells, cell_y + radius_y + 1)
                entrance_free = belief[y0:y1, x0:x1] == FREE
                traversable[y0:y1, x0:x1] |= entrance_free

        return traversable

    @staticmethod
    def _frontier_mask(belief):
        unknown = belief == UNKNOWN
        free = belief == FREE
        adjacent_unknown = np.zeros_like(unknown, dtype=bool)
        adjacent_unknown[1:, :] |= unknown[:-1, :]
        adjacent_unknown[:-1, :] |= unknown[1:, :]
        adjacent_unknown[:, 1:] |= unknown[:, :-1]
        adjacent_unknown[:, :-1] |= unknown[:, 1:]
        return free & adjacent_unknown

    def _exploration_frontier_mask(self, belief):
        if self.exploration_roi is None:
            return self._frontier_mask(belief)
        bounded_belief = belief.copy()
        bounded_belief[~self.exploration_roi] = OCCUPIED
        return self._frontier_mask(bounded_belief)

    def _astar_path(self, belief, start, goal, allow_unknown, restrict_roi=False):
        start_cell = self._cell(start)
        goal_cell = self._cell(goal)
        if start_cell is None or goal_cell is None:
            return []
        if allow_unknown:
            # Unknown space is passable, but keep the robot's physical footprint
            # clear of known obstacles so a doorway/interior path centres the
            # body in the opening instead of grazing the frame.  With
            # robot_radius_cells == 0 this degrades to the old point-robot rule.
            occupied = belief == OCCUPIED
            blocked = self._dilate_mask(occupied, self.robot_radius_cells)
            traversable = ~blocked
        else:
            traversable = self._known_free_mask(belief)
        if restrict_roi and self.exploration_roi is not None:
            traversable &= self.exploration_roi
        traversable[start_cell[1], start_cell[0]] = True
        traversable[goal_cell[1], goal_cell[0]] = True
        heap = [(0.0, start_cell)]
        g_score = {start_cell: 0.0}
        parent = {start_cell: None}
        visited_count = 0
        while heap and visited_count < 30000:
            _, current = heapq.heappop(heap)
            visited_count += 1
            if current == goal_cell:
                cells = []
                while current is not None:
                    cells.append(current)
                    current = parent[current]
                cells.reverse()
                return self._cells_to_waypoints(cells)
            cx, cy = current
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if not self._inside(nx, ny) or not traversable[ny, nx]:
                    continue
                step = 1.0
                candidate = (nx, ny)
                new_g = g_score[current] + step
                if new_g < g_score.get(candidate, float("inf")):
                    g_score[candidate] = new_g
                    heuristic = math.hypot(nx - goal_cell[0], ny - goal_cell[1])
                    heapq.heappush(heap, (new_g + heuristic, candidate))
                    parent[candidate] = current
        return []

    def _near_obstacle(self, belief, x, y, radius):
        x0, x1 = max(0, x - radius), min(self.map_cells, x + radius + 1)
        y0, y1 = max(0, y - radius), min(self.map_cells, y + radius + 1)
        return bool(np.any(belief[y0:y1, x0:x1] == OCCUPIED))

    def _cells_to_waypoints(self, cells):
        if not cells:
            return []
        waypoints = []
        last = None
        for index, cell in enumerate(cells):
            if index == 0:
                continue
            turn = False
            if 0 < index < len(cells) - 1:
                previous_delta = (
                    cells[index][0] - cells[index - 1][0],
                    cells[index][1] - cells[index - 1][1],
                )
                next_delta = (
                    cells[index + 1][0] - cells[index][0],
                    cells[index + 1][1] - cells[index][1],
                )
                turn = previous_delta != next_delta
            if index not in (1, len(cells) - 1) and index % 3 != 0 and not turn:
                continue
            point = (self.map_origin_x + cell[0] * self.cell_size,
                     self.map_origin_y + cell[1] * self.cell_size)
            distance_from_last = (
                float("inf") if last is None
                else math.hypot(point[0] - last[0], point[1] - last[1])
            )
            if distance_from_last >= 0.5 or index in (1, len(cells) - 1) or turn:
                if last is not None and distance_from_last < 1e-6:
                    continue
                waypoints.append(point)
                last = point
        return waypoints

    def _set_path(self, path, status):
        with self.lock:
            # A planning callback may finish with a stale snapshot after the
            # control callback has already stopped at home.
            if not self.active:
                return False
            self.path = path
            self.path_index = 0
            self.last_plan = rospy.Time.now()
            self.status = status
            if self.pose is not None:
                self.visited_by_floor[self.current_floor].add(
                    (round(self.pose[0], 1), round(self.pose[1], 1))
                )
                if self.map_origin_x is not None:
                    cell = self._cell((self.pose[0], self.pose[1]))
                    if cell is not None:
                        self.visited_cells[self.current_floor].add(cell)
        return True

    def control_timer(self, _event):
        command = Twist()
        waypoint = None
        home_tolerance = 0.35
        with self.lock:
            pose = self.pose
            active = self.active
            path = list(self.path)
            index = self.path_index
            mode = self.mode
            status = self.status
        waypoint_tolerance = self._waypoint_tolerance(mode)
        if not active or pose is None:
            self.cmd_pub.publish(command)
            return
        # Generic path-progress stall guard: while following a path (not on
        # stairs, where slow progress is normal), if the pose has not advanced
        # for ctrl_stall_timeout seconds, drop the path so the planning timer
        # re-plans.  Without this, a collision_safety hold against an obstacle
        # leaves the robot pushing at full speed into the wall forever.
        if (index < len(path) and mode not in ("stairs_up", "stairs_down")):
            moved = 0.0
            if self.ctrl_last_pose is not None:
                moved = math.hypot(
                    pose[0] - self.ctrl_last_pose[0],
                    pose[1] - self.ctrl_last_pose[1],
                )
            now = rospy.Time.now()
            if (self.ctrl_last_time == rospy.Time(0) or
                    moved >= 0.15):
                self.ctrl_last_pose = pose
                self.ctrl_last_time = now
            elif now - self.ctrl_last_time > rospy.Duration(self.ctrl_stall_timeout):
                with self.lock:
                    self.path = []
                    self.path_index = 0
                    self.status = "path_stalled_replan"
                self.ctrl_last_pose = pose
                self.ctrl_last_time = now
                rospy.logwarn_throttle(
                    5.0, "path progress stalled; dropping path for re-plan"
                )
        else:
            self.ctrl_last_pose = pose
            self.ctrl_last_time = rospy.Time(0)
        if rospy.Time.now() - self.last_map_update > rospy.Duration(5.0) and mode != "return":
            self.cmd_pub.publish(command)
            rospy.logwarn_throttle(5.0, "scan map is stale; holding zero velocity")
            return
        if mode == "return":
            with self.lock:
                home = self.home
            if home is not None and math.hypot(pose[0] - home[0], pose[1] - home[1]) <= home_tolerance:
                with self.lock:
                    self.active = False
                    self.mode = "idle"
                    self.status = "returned_home full_map=%s" % self.full_map_complete
                self.cmd_pub.publish(command)
                rospy.loginfo("return-home completed")
                return
        while (index < len(path) and
               math.hypot(path[index][0] - pose[0],
                          path[index][1] - pose[1]) < waypoint_tolerance):
            index += 1
        if index >= len(path):
            if mode in ("stairs_up", "stairs_down"):
                with self.lock:
                    self.path_index = len(path)
                self.cmd_pub.publish(command)
                return
            if mode == "explore" and status.startswith("survey_"):
                command.angular.z = min(0.8, self.max_yaw_rate)
                with self.lock:
                    self.last_command = command
                self.cmd_pub.publish(command)
                return
            if mode == "return":
                home_distance = float("inf")
                with self.lock:
                    if self.home is not None:
                        home_distance = math.hypot(pose[0] - self.home[0], pose[1] - self.home[1])
                if home_distance > home_tolerance:
                    with self.lock:
                        self.status = "return_waiting_for_path"
                    self.cmd_pub.publish(command)
                    return
                with self.lock:
                    self.active = False
                    self.mode = "idle"
                    self.status = "returned_home full_map=%s" % self.full_map_complete
                self.cmd_pub.publish(command)
                rospy.loginfo("return-home completed")
                return
            with self.lock:
                self.path_index = 0
            self.cmd_pub.publish(command)
            return

        waypoint = path[index]
        tracking_waypoint = self._tracking_waypoint(
            mode, index, pose, waypoint, self.stair_lookahead
        )
        dx = tracking_waypoint[0] - pose[0]
        dy = tracking_waypoint[1] - pose[1]
        body_x = math.cos(pose[3]) * dx + math.sin(pose[3]) * dy
        body_y = -math.sin(pose[3]) * dx + math.cos(pose[3]) * dy
        target_yaw = math.atan2(dy, dx)
        heading_error = target_yaw - pose[3]
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        command.linear.x = clamp(self.linear_gain * body_x, -self.max_linear, self.max_linear)
        command.linear.y = clamp(0.55 * body_y, -self.max_lateral, self.max_lateral)
        if abs(heading_error) > 1.0:
            command.linear.x *= 0.2
            command.linear.y *= 0.2
        command.angular.z = clamp(0.9 * heading_error, -self.max_yaw_rate, self.max_yaw_rate)
        if mode in ("stairs_up", "stairs_down"):
            waypoint_distance = math.hypot(
                waypoint[0] - pose[0], waypoint[1] - pose[1]
            )
            cruise_command = self._stair_cruise_command(
                mode,
                index,
                body_x,
                body_y,
                heading_error,
                waypoint_distance,
                self.stair_speed,
                self.stair_brake_distance,
                self.stair_cruise_heading_tolerance,
                self.stair_cruise_lateral_tolerance,
            )
            if cruise_command is not None:
                command.linear.x = cruise_command
            elif index in (1, 3) and (
                abs(heading_error) > self.stair_cruise_heading_tolerance or
                abs(body_y) > self.stair_cruise_lateral_tolerance
            ):
                # Misaligned on a flight segment: hold FORWARD motion and
                # rotate + strafe in place (angular.z and linear.y below stay
                # active) until heading/lateral error fall inside the cruise
                # window.  Driving forward while misaligned slips the robot off
                # the stair edge.  NOTE: aligned-but-braking (waypoint within
                # brake distance) is intentionally left to the normal controller
                # so the robot keeps creeping forward to reach the waypoint --
                # stalling mid-stairs is itself a fall trigger.
                command.linear.x = 0.0
            # Turn on a landing before advancing. Carrying forward motion
            # through a 90-degree stair turn is a common source of slip.
            if index not in (1, 3) and abs(heading_error) > self.stair_turn_heading_threshold:
                command.linear.x = 0.0
                command.linear.y = 0.0
            speed_cap = self._stair_speed_cap(
                mode,
                index,
                self.stair_speed,
                self.stair_landing_speed,
                self.max_linear,
                self.stair_flat_use_max_linear,
            )
            command.linear.x = clamp(command.linear.x, -speed_cap, speed_cap)
            command.linear.y = clamp(
                command.linear.y, -self.stair_lateral_limit, self.stair_lateral_limit
            )
            yaw_rate_limit = self._yaw_rate_limit(
                mode, self.max_yaw_rate, self.stair_yaw_rate
            )
            command.angular.z = clamp(command.angular.z,
                                      -yaw_rate_limit, yaw_rate_limit)
        if not all(math.isfinite(value) for value in
                   (command.linear.x, command.linear.y, command.angular.z)):
            rospy.logerr("computed cmd_vel contains NaN/Inf; sending zero")
            command = Twist()
        with self.lock:
            self.path_index = index
            self.last_command = command
        self.cmd_pub.publish(command)
        if waypoint is not None:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = self.map_frame
            pose_msg.pose.position.x = waypoint[0]
            pose_msg.pose.position.y = waypoint[1]
            pose_msg.pose.orientation.w = 1.0
            self.waypoint_pub.publish(pose_msg)

    def publish_zero(self):
        self.cmd_pub.publish(Twist())

    def _occupancy_message(self, belief, floor_index):
        message = OccupancyGrid()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.map_frame
        message.info.resolution = self.cell_size
        message.info.width = self.map_cells
        message.info.height = self.map_cells
        message.info.origin.position.x = self.map_origin_x
        message.info.origin.position.y = self.map_origin_y
        if self.home is not None:
            message.info.origin.position.z = self.home[2] + floor_index * self.floor_height
        message.info.origin.orientation.w = 1.0
        values = np.full(belief.shape, -1, dtype=np.int8)
        values[belief == FREE] = 0
        values[belief == OCCUPIED] = 100
        message.data = values.reshape(-1).tolist()
        return message

    def publish_timer(self, _event):
        with self.lock:
            coverage = [self._coverage_ratio(item) for item in self.floor_beliefs]
            self.floor_coverage = coverage
            graph_rooms = [
                sum(
                    1 for node in graph.nodes.values()
                    if node.node_type == "room_entry"
                )
                for graph in self.floor_graphs
            ]
            room_tasks = self._room_task_status()
            status = ("%s mode=%s active=%s policy=%s inferences=%d "
                      "floor=%d/%d full_map=%s complete=%s coverage=%s "
                      "frontiers=%s graph_rooms=%s distance=%s pose=%s "
                      "home=%s path=%d/%d room_tasks=%s") % (
                self.status,
                self.mode,
                self.active,
                self.policy.reason,
                self.policy.inference_count,
                self.current_floor,
                self.floor_count,
                self.full_map_complete,
                ",".join("1" if item else "0" for item in self.floor_complete),
                ",".join("%.3f" % item for item in coverage),
                ",".join(str(item) for item in self.reachable_frontiers),
                ",".join(str(item) for item in graph_rooms),
                ",".join("%.1f" % item for item in self.floor_distance),
                self.pose,
                self.home,
                self.path_index,
                len(self.path),
                room_tasks,
            )
            origin_x, origin_y = self.map_origin_x, self.map_origin_y
            current_floor = self.current_floor
            beliefs = [item.copy() for item in self.floor_beliefs]
        self.status_pub.publish(String(status))
        if origin_x is None:
            return
        for floor_index, belief in enumerate(beliefs):
            message = self._occupancy_message(belief, floor_index)
            self.floor_map_pubs[floor_index].publish(message)
            if floor_index == current_floor:
                self.map_pub.publish(message)


def main():
    rospy.init_node("competition_navigation")
    CompetitionNavigation()
    rospy.spin()


if __name__ == "__main__":
    main()
