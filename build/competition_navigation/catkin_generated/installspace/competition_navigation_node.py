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
import json
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
        self.scan_stride = max(1, int(rospy.get_param("~scan_stride", 8)))
        self.max_scan_points = max(100, int(rospy.get_param("~max_scan_points", 1800)))
        self.map_progress_cell_batch = max(
            1, int(rospy.get_param("~map_progress_cell_batch", 8))
        )
        self.max_linear = float(rospy.get_param("~max_linear", 0.5))
        self.linear_gain = float(rospy.get_param("~linear_gain", 1.0))
        self.max_lateral = float(rospy.get_param("~max_lateral", 0.15))
        self.max_yaw_rate = float(rospy.get_param("~max_yaw_rate", 1.2))
        self.allow_unknown_return = bool(rospy.get_param("~allow_unknown_return", False))
        self.auto_start = bool(rospy.get_param("~auto_start", False))
        self.auto_return_after_sec = float(rospy.get_param("~auto_return_after_sec", 0.0))
        self.require_hdplanner = bool(rospy.get_param("~require_hdplanner", True))
        self.layout_metadata_path = rospy.get_param(
            "~layout_metadata_path",
            "/home/uf/SimEnv/generated_building/layout_metadata.json",
        )
        self.floor_count = max(1, int(rospy.get_param("~floor_count", 3)))
        self.floor_height = float(rospy.get_param("~floor_height", 2.6))
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
        self.stair_speed = float(rospy.get_param("~stair_speed", 0.40))
        self.min_room_coverage = max(
            0.50, float(rospy.get_param("~min_room_coverage", 0.70))
        )
        self.room_task_timeout = max(
            30.0, float(rospy.get_param("~room_task_timeout", 120.0))
        )
        self.stair_bounds = None
        self.corridor_bounds = None
        self.room_door_ys = []
        self.expected_room_count = []
        self.footprint_bounds = None
        self._load_layout_configuration()
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
        rospy.Timer(rospy.Duration(1.0), self.publish_timer)
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

    def _load_layout_configuration(self):
        """Load only building infrastructure, never room or danger-source truth."""
        try:
            with open(self.layout_metadata_path, "r") as stream:
                metadata = json.load(stream)
            floors = metadata.get("floors", [])
            if floors:
                self.floor_count = len(floors)
                self.expected_room_count = [
                    len(floor.get("rooms", [])) for floor in floors
                ]
                self.stair_bounds = dict(floors[0].get("stair_bounds", {}))
                self.corridor_bounds = dict(
                    floors[0].get("corridor_bounds", {})
                )
                self.room_door_ys = sorted(
                    float(room["door_pose"][1])
                    for room in floors[0].get("rooms", [])
                    if len(room.get("door_pose", [])) >= 2
                )
                self.lobby_bounds = dict(floors[0].get("lobby_bounds", {}))
                self.elevator_bounds = dict(
                    floors[0].get("elevator_bounds", {})
                )
            self.floor_height = float(metadata.get("floor_height", self.floor_height))
            footprint = metadata.get("footprint", {})
            entrance = metadata.get("entrance_pose", [0.0, 0.0])
            width = float(footprint.get("width", 0.0))
            length = float(footprint.get("length", 0.0))
            if width > 0.0 and length > 0.0:
                entrance_x = float(entrance[0])
                entrance_y = float(entrance[1])
                self.footprint_bounds = {
                    "x_min": entrance_x - width / 2.0,
                    "x_max": entrance_x + width / 2.0,
                    "y_min": entrance_y,
                    "y_max": entrance_y + length,
                }
            rospy.loginfo(
                "loaded competition infrastructure: floors=%d height=%.2f footprint=%s stair=%s",
                self.floor_count,
                self.floor_height,
                self.footprint_bounds,
                self.stair_bounds,
            )
        except Exception as exc:
            rospy.logwarn(
                "layout metadata unavailable (%s); using configured floor defaults", exc
            )
        if self.floor_count > 1 and not self.stair_bounds:
            self.stair_bounds = {
                "x_min": -4.85,
                "x_max": -1.65,
                "y_min": 0.85,
                "y_max": 6.85,
            }

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
                    opening = (
                        belief[wall_cell[1], wall_cell[0]] == FREE and
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
                    if len(run) >= 2:
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

    def _update_floor_graph(self, belief, floor_index):
        """Record observed corridor anchors and side-room entry nodes."""
        if floor_index >= len(self.floor_graphs):
            return
        graph = self.floor_graphs[floor_index]
        root_id = self._graph_root(floor_index)
        candidates = self._room_entry_candidates(belief)
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

    def _room_frontier_path(self, belief, pose, node):
        """Return a known-free path to a frontier inside one room."""
        bounds = self._room_bounds(node)
        if bounds is None:
            return []
        room_mask = self._room_mask(bounds)
        frontier = self._frontier_mask(belief) & room_mask
        traversable = self._known_free_mask(belief) & room_mask
        start = self._cell((pose[0], pose[1]))
        if start is None:
            return []
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
                next_x, next_y = cell_x + dx, cell_y + dy
                if (not self._inside(next_x, next_y) or
                        not traversable[next_y, next_x] or
                        distance[next_y, next_x] >= 0):
                    continue
                distance[next_y, next_x] = next_distance
                parent_x[next_y, next_x] = cell_x
                parent_y[next_y, next_x] = cell_y
                queue.append((next_x, next_y))
        candidates = np.argwhere(frontier & (distance >= 2))
        if candidates.size == 0:
            return []
        unknown = belief == UNKNOWN
        best_cell = None
        best_score = -float("inf")
        for cell_y, cell_x in candidates:
            radius = 4
            y0 = max(0, cell_y - radius)
            y1 = min(self.map_cells, cell_y + radius + 1)
            x0 = max(0, cell_x - radius)
            x1 = min(self.map_cells, cell_x + radius + 1)
            information_gain = int(np.count_nonzero(unknown[y0:y1, x0:x1]))
            score = information_gain - 0.15 * float(distance[cell_y, cell_x])
            if score > best_score:
                best_score = score
                best_cell = (int(cell_x), int(cell_y))
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
                # omitted from DFS.
                if not self._prepare_graph_dfs(floor_index):
                    self.graph_phase[floor_index] = "frontier_fallback"
                    self.status = "room_graph_unavailable"
                    return False
                order = self.graph_dfs_order[floor_index]

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
                return True
            path = self._room_path(belief, pose, node, entering=True)
            if path:
                self._set_path(path, "room_enter_path id=%d" % node.node_id)
                return True
            if elapsed >= self.room_task_timeout:
                graph.mark_node(self.active_room_node, blocked=True)
                self.status = "room_blocked id=%d" % node.node_id
                self.active_room_node = None
                self.active_room_phase = None
                self.graph_dfs_index[floor_index] += 1
                return True
            self.status = "room_waiting_for_entry_path id=%d" % node.node_id
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
                    return True
                path = self._room_frontier_path(belief, pose, node)
                if path:
                    self.active_room_no_frontier_cycles = 0
                    self._set_path(path, "room_frontier id=%d %.3f" % (
                        node.node_id, coverage
                    ))
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
        door_ys = list(self.room_door_ys)
        if not door_ys:
            door_ys = [y_min + 0.25 * (y_max - y_min),
                       y_min + 0.75 * (y_max - y_min)]
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
            new_known_cells = 0
            for point in transformed:
                px, py, pz = point
                if not all(math.isfinite(value) for value in (px, py, pz)):
                    continue
                distance = math.hypot(px - pose[0], py - pose[1])
                if distance > 22.0 or distance < 2.0:
                    continue
                # Keep the current floor and reject high ceiling returns.
                if pz < pose[2] - 0.05 or pz > pose[2] + 1.0:
                    continue
                endpoint = self._cell((px, py), origin_x, origin_y)
                if endpoint is None:
                    continue
                line = list(bresenham(sensor_cell[0], sensor_cell[1], endpoint[0], endpoint[1]))
                for cell_x, cell_y in line[:-1]:
                    if self._inside(cell_x, cell_y):
                        if belief[cell_y, cell_x] != OCCUPIED:
                            if (belief[cell_y, cell_x] == UNKNOWN and
                                    (self.exploration_roi is None or
                                     self.exploration_roi[cell_y, cell_x])):
                                new_known_cells += 1
                            belief[cell_y, cell_x] = FREE
                ex, ey = endpoint
                if self._inside(ex, ey):
                    if (belief[ey, ex] == UNKNOWN and
                            (self.exploration_roi is None or
                             self.exploration_roi[ey, ex])):
                        new_known_cells += 1
                    belief[ey, ex] = OCCUPIED
            if self._inside(robot_cell[0], robot_cell[1]):
                if (belief[robot_cell[1], robot_cell[0]] == UNKNOWN and
                        (self.exploration_roi is None or
                         self.exploration_roi[robot_cell[1], robot_cell[0]])):
                    new_known_cells += 1
                belief[robot_cell[1], robot_cell[0]] = FREE
            self.pending_known_cells[floor_index] += new_known_cells
            if (self.pending_known_cells[floor_index] >=
                    self.map_progress_cell_batch):
                self.last_progress_pose = pose
                self.last_progress_time = rospy.Time.now()
                self.pending_known_cells[floor_index] = 0
            self.latest_scan_stamp = msg.header.stamp
            self.last_map_update = rospy.Time.now()

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
        """Require every discovered room on a floor to finish before exit."""
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
        return bool(rooms) and all(node.completed for node in rooms)

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
            for cell_y, cell_x in candidates:
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
    def _yaw_rate_limit(mode, max_yaw_rate):
        if mode in ("stairs_up", "stairs_down"):
            return min(max_yaw_rate, 0.8)
        return max_yaw_rate

    @staticmethod
    def _tracking_waypoint(mode, path_index, pose, waypoint):
        if mode not in ("stairs_up", "stairs_down") or path_index not in (1, 3):
            return waypoint
        delta_y = waypoint[1] - pose[1]
        # Descending must stay on the next physical tread; a long lookahead
        # skips several risers and can wedge the base against the stair edge.
        lookahead = 1.20
        if abs(delta_y) <= lookahead:
            return waypoint
        return (waypoint[0], pose[1] + math.copysign(lookahead, delta_y))

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
        observed_free = self._dilate_mask(free, 2)
        blocked = self._dilate_mask(occupied, 1)
        traversable = observed_free & ~blocked

        occupied_up = np.zeros_like(occupied, dtype=bool)
        occupied_down = np.zeros_like(occupied, dtype=bool)
        occupied_left = np.zeros_like(occupied, dtype=bool)
        occupied_right = np.zeros_like(occupied, dtype=bool)
        for offset in (1, 2):
            occupied_up[offset:, :] |= occupied[:-offset, :]
            occupied_down[:-offset, :] |= occupied[offset:, :]
            occupied_left[:, offset:] |= occupied[:, :-offset]
            occupied_right[:, :-offset] |= occupied[:, offset:]
        doorway_seed = free & (
            (occupied_up & occupied_down) |
            (occupied_left & occupied_right)
        )
        doorway_clearance = self._dilate_mask(doorway_seed, 2)
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
            traversable = belief != OCCUPIED
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
            mode, index, pose, waypoint
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
            command.linear.x = clamp(command.linear.x, -self.stair_speed, self.stair_speed)
            command.linear.y = clamp(command.linear.y, -0.10, 0.10)
            yaw_rate_limit = self._yaw_rate_limit(mode, self.max_yaw_rate)
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
