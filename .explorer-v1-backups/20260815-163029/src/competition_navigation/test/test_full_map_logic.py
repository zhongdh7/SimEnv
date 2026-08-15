#!/usr/bin/env python3

import os
import sys
import threading
import unittest
from types import SimpleNamespace
from unittest import mock

import numpy as np


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "scripts"))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "tools"))

import competition_navigation_node as navigation_module
from competition_navigation_node import CompetitionNavigation, FREE, OCCUPIED, UNKNOWN
from full_map_verifier import FullMapVerifier
from corridor_graph import CorridorGraph


class _PolicyStub(object):
    K_SIZE = 25


def _navigation_stub(size=60):
    navigation = CompetitionNavigation.__new__(CompetitionNavigation)
    navigation.map_cells = size
    navigation.cell_size = 0.4
    navigation.map_origin_x = 0.0
    navigation.map_origin_y = 0.0
    navigation.exploration_roi = np.ones((size, size), dtype=bool)
    navigation.blacklisted_frontiers = [set(), set(), set()]
    navigation.visited_by_floor = [set(), set(), set()]
    navigation.policy = _PolicyStub()
    navigation.stair_bounds = {
        "x_min": -4.85,
        "x_max": -1.65,
        "y_min": 0.85,
        "y_max": 6.85,
    }
    navigation.corridor_bounds = {
        "x_min": -1.1,
        "x_max": 1.1,
        "y_min": 7.85,
        "y_max": 35.91,
    }
    navigation.room_door_ys = [14.865, 28.895]
    navigation.footprint_bounds = {
        "x_min": 0.0,
        "x_max": size * navigation.cell_size,
        "y_min": 0.0,
        "y_max": size * navigation.cell_size,
    }
    navigation.min_floor_coverage = 0.45
    navigation.completion_min_floor_coverage = 0.70
    navigation.completion_min_sector_coverage = 0.30
    navigation.completion_min_floor_distance = 35.0
    navigation.floor_distance = [0.0, 0.0, 0.0]
    navigation.floor_graphs = [CorridorGraph(), CorridorGraph(), CorridorGraph()]
    navigation.expected_room_count = [0, 0, 0]
    navigation.active_room_node = None
    room_id, _ = navigation.floor_graphs[0].add_room_entry(
        0, -1.1, 14.865, corridor_s=14.865
    )
    navigation.floor_graphs[0].mark_node(room_id, completed=True)
    return navigation


class FullMapLogicTest(unittest.TestCase):
    def test_reachable_frontier_builds_hdplanner_graph(self):
        navigation = _navigation_stub()
        belief = np.full((60, 60), UNKNOWN, dtype=np.uint8)
        belief[5:45, 5:45] = FREE
        pose = (4.0, 4.0, 0.4, 0.0)

        path, target, reachable = navigation._reachable_frontier_plan(
            belief, pose, 0
        )

        self.assertGreater(reachable, 0)
        self.assertGreaterEqual(len(path), 2)
        self.assertIsNotNone(target)
        graph = navigation._build_graph(belief, pose, target, 0, path)
        nodes, _, _, current, target_index, neighbors, centers, _ = graph
        self.assertEqual(current, 0)
        self.assertNotEqual(target_index, current)
        self.assertGreater(len(neighbors), 0)
        self.assertIn(target_index, centers)
        self.assertLess(np.linalg.norm(nodes[target_index] - target), 0.21)

    def test_reachable_frontier_keeps_current_reachable_target(self):
        navigation = _navigation_stub()
        navigation.current_frontier_cell = (5, 10)
        belief = np.full((60, 60), UNKNOWN, dtype=np.uint8)
        belief[5:45, 5:45] = FREE

        path, target, reachable = navigation._reachable_frontier_plan(
            belief, (4.0, 4.0, 0.4, 0.0), 0
        )

        self.assertGreater(reachable, 0)
        self.assertTrue(path)
        self.assertEqual(target, (2.0, 4.0))

    def test_graph_omits_lattice_node_near_current_pose(self):
        navigation = _navigation_stub()
        navigation.map_origin_x = 0.25
        navigation.map_origin_y = 0.25
        belief = np.full((60, 60), FREE, dtype=np.uint8)
        pose = (4.05, 4.05, 0.4, 0.0)

        graph = navigation._build_graph(
            belief, pose, (12.0, 4.0), 0, [(5.25, 4.05), (6.05, 4.05)]
        )
        nodes = graph[0]

        distances = [np.linalg.norm(node - nodes[0]) for node in nodes[1:]]
        self.assertTrue(distances)
        self.assertGreaterEqual(min(distances), 0.8)

    def test_floor_estimation_uses_home_height(self):
        navigation = _navigation_stub()
        navigation.home = (0.0, 0.0, 0.4, 0.0)
        navigation.floor_height = 2.6
        navigation.floor_count = 3
        self.assertEqual(navigation._floor_from_height(0.4), 0)
        self.assertEqual(navigation._floor_from_height(3.0), 1)
        self.assertEqual(navigation._floor_from_height(5.6), 2)

    def test_stair_routes_follow_opposite_u_shapes(self):
        navigation = _navigation_stub()
        up = navigation._stair_route("up")
        down = navigation._stair_route("down")
        self.assertEqual(len(up), 5)
        self.assertEqual(len(down), 5)
        self.assertEqual(up[0], down[-2])
        self.assertEqual(up[1], down[-3])
        self.assertEqual(up[2], down[1])
        self.assertEqual(up[3], down[0])
        self.assertEqual(up[-1], down[-1])
        self.assertGreater(up[-1][0], navigation.stair_bounds["x_max"])

    def test_stair_run_tracking_uses_moving_centerline_lookahead(self):
        navigation = _navigation_stub()
        pose = (-3.10, 2.00, 0.4, 0.0)
        endpoint = (-4.018, 5.20)

        tracking = navigation._tracking_waypoint(
            "stairs_up", 1, pose, endpoint
        )

        self.assertEqual(tracking, (-4.018, 3.20))
        self.assertEqual(
            navigation._tracking_waypoint("stairs_up", 2, pose, endpoint),
            endpoint,
        )
        self.assertEqual(
            navigation._tracking_waypoint("explore", 1, pose, endpoint),
            endpoint,
        )

    def test_completed_stair_path_holds_for_transition_confirmation(self):
        navigation = _navigation_stub()
        navigation.lock = threading.RLock()
        navigation.path = navigation._stair_route("up")
        endpoint = navigation.path[-1]
        navigation.pose = (endpoint[0], endpoint[1], 2.9, 0.0)
        navigation.active = True
        navigation.path_index = len(navigation.path) - 1
        navigation.mode = "stairs_up"
        navigation.status = "stairs_up_started target_floor=1"
        navigation.last_map_update = navigation_module.rospy.Time.from_sec(10.0)
        published = []
        navigation.cmd_pub = SimpleNamespace(publish=published.append)

        with mock.patch(
            "competition_navigation_node.rospy.Time.now",
            return_value=navigation_module.rospy.Time.from_sec(10.1),
        ):
            navigation.control_timer(None)

        self.assertEqual(navigation.path_index, len(navigation.path))
        self.assertEqual(len(published), 1)
        self.assertEqual(published[0].linear.x, 0.0)
        self.assertEqual(published[0].linear.y, 0.0)
        self.assertEqual(published[0].angular.z, 0.0)

    def test_stair_entry_uses_open_east_side(self):
        navigation = _navigation_stub()
        geometry = navigation._stair_geometry()
        expected_entry = (geometry[4], geometry[2])

        self.assertEqual(navigation._stair_entry("up"), expected_entry)
        self.assertEqual(navigation._stair_entry("down"), expected_entry)
        self.assertGreater(expected_entry[0], navigation.stair_bounds["x_max"])

    def test_stair_entry_route_uses_nearest_room_door(self):
        navigation = _navigation_stub()
        path = navigation._safe_stair_entry_path((1.6, 28.2, 0.4, 0.0))

        self.assertAlmostEqual(path[0][0], 1.6)
        self.assertAlmostEqual(path[0][1], 28.895)
        self.assertAlmostEqual(path[1][0], 0.65)
        self.assertAlmostEqual(path[1][1], 28.895)
        self.assertEqual(path[-1], navigation._stair_entry("up"))

    def test_stair_entry_route_aligns_nearby_pose_before_crossing_door(self):
        navigation = _navigation_stub()
        path = navigation._safe_stair_entry_path((6.0, 14.74, 0.4, 0.0))

        self.assertGreater(path[0][1], 14.865)
        self.assertAlmostEqual(path[1][1], 14.865)

    def test_stair_entry_route_does_not_return_to_corridor_from_lobby(self):
        navigation = _navigation_stub()
        path = navigation._safe_stair_entry_path((0.0, 6.2, 0.4, 0.0))

        self.assertEqual(path, [navigation._stair_entry("up")])

    def test_stair_waypoint_tolerance_is_tighter_than_regular_navigation(self):
        navigation = _navigation_stub()

        self.assertAlmostEqual(navigation._waypoint_tolerance("explore"), 0.35)
        self.assertAlmostEqual(navigation._waypoint_tolerance("stairs_up"), 0.25)
        self.assertAlmostEqual(navigation._waypoint_tolerance("stairs_down"), 0.25)

    def test_stair_transition_does_not_fuse_scan_into_floor_map(self):
        navigation = _navigation_stub()
        navigation.lock = threading.RLock()
        navigation.pose = (4.0, 4.0, 2.9, 0.0)
        navigation.current_floor = 1
        navigation.mode = "stairs_up"
        navigation.floor_beliefs = [
            np.full((60, 60), UNKNOWN, dtype=np.uint8) for _ in range(3)
        ]
        navigation.scan_stride = 1
        navigation.max_scan_points = 100
        navigation.pending_known_cells = [0, 0, 0]
        navigation.map_progress_cell_batch = 1000
        navigation._transform_points = lambda *_args: [(8.0, 4.0, 2.9)]
        navigation._sensor_origin = lambda *_args: (4.0, 4.0, 2.9)
        message = SimpleNamespace(
            points=[SimpleNamespace(x=4.0, y=0.0, z=0.0)],
            header=SimpleNamespace(frame_id="laser_livox", stamp=0),
        )

        with mock.patch(
            "competition_navigation_node.rospy.Time.now", return_value=0
        ):
            navigation.scan_callback(message)

        self.assertTrue(
            np.all(navigation.floor_beliefs[1] == UNKNOWN)
        )

    def test_stair_yaw_limit_overcomes_platform_turning_deadband(self):
        navigation = _navigation_stub()

        self.assertAlmostEqual(
            navigation._yaw_rate_limit("stairs_up", 1.2), 0.8
        )
        self.assertAlmostEqual(
            navigation._yaw_rate_limit("stairs_down", 0.6), 0.6
        )
        self.assertAlmostEqual(
            navigation._yaw_rate_limit("explore", 1.2), 1.2
        )

    def test_coverage_ratio_is_limited_to_exploration_roi(self):
        navigation = _navigation_stub(size=10)
        navigation.exploration_roi.fill(False)
        navigation.exploration_roi[2:8, 2:8] = True
        belief = np.full((10, 10), UNKNOWN, dtype=np.uint8)
        belief[2:5, 2:8] = FREE
        self.assertAlmostEqual(navigation._coverage_ratio(belief), 0.5)

    def test_distributed_coverage_requires_every_footprint_sector(self):
        navigation = _navigation_stub(size=40)
        navigation.floor_distance[0] = 40.0
        belief = np.full((40, 40), UNKNOWN, dtype=np.uint8)
        belief[:, :24] = FREE

        self.assertFalse(
            navigation._distributed_coverage_complete(belief, 1.0, 0)
        )

        belief[:, 24:] = FREE
        self.assertTrue(
            navigation._distributed_coverage_complete(belief, 1.0, 0)
        )

    def test_distributed_coverage_requires_minimum_travel(self):
        navigation = _navigation_stub(size=40)
        belief = np.full((40, 40), FREE, dtype=np.uint8)
        navigation.floor_distance[0] = 34.9

        self.assertFalse(
            navigation._distributed_coverage_complete(belief, 1.0, 0)
        )

    def test_distributed_coverage_rejects_low_total_coverage(self):
        navigation = _navigation_stub(size=40)
        navigation.floor_distance[0] = 80.0
        belief = np.full((40, 40), FREE, dtype=np.uint8)

        self.assertFalse(
            navigation._distributed_coverage_complete(belief, 0.699, 0)
        )

    def test_distributed_coverage_requires_completed_room_tasks(self):
        navigation = _navigation_stub(size=40)
        navigation.floor_distance[0] = 80.0
        belief = np.full((40, 40), FREE, dtype=np.uint8)
        room_id = next(iter(navigation.floor_graphs[0].nodes))
        navigation.floor_graphs[0].mark_node(room_id, completed=False)

        self.assertFalse(
            navigation._distributed_coverage_complete(belief, 1.0, 0)
        )

    def test_verifier_parses_aggregated_room_task_evidence(self):
        status = (
            "room_complete id=2 room_tasks="
            "0:2:L:14.87:1:1:1:0:0.82,1:4:R:28.90:1:1:0:1:0.31"
        )
        tasks = FullMapVerifier._parse_room_tasks(status)

        self.assertEqual(len(tasks), 2)
        self.assertTrue(tasks[0]["entered"])
        self.assertTrue(tasks[0]["interior_observation"])
        self.assertTrue(tasks[0]["completed"])
        self.assertTrue(tasks[1]["blocked"])

    def test_exploration_roi_connects_spawn_to_main_entrance(self):
        navigation = _navigation_stub(size=200)
        navigation.map_origin_x = -40.0
        navigation.map_origin_y = -42.2
        navigation.home = (0.0, -2.2, 0.4, 0.0)
        navigation.footprint_bounds = {
            "x_min": -10.0,
            "x_max": 10.0,
            "y_min": 0.0,
            "y_max": 36.0,
        }
        navigation._build_exploration_roi()
        home_cell = navigation._cell((0.0, -2.2))
        entrance_cell = navigation._cell((0.0, 0.0))
        outdoor_cell = navigation._cell((8.0, -2.2))
        self.assertTrue(navigation.exploration_roi[home_cell[1], home_cell[0]])
        self.assertTrue(
            navigation.exploration_roi[entrance_cell[1], entrance_cell[0]]
        )
        self.assertFalse(
            navigation.exploration_roi[outdoor_cell[1], outdoor_cell[0]]
        )

    def test_exploration_roi_excludes_stairs_but_keeps_open_entry(self):
        navigation = _navigation_stub(size=200)
        navigation.map_origin_x = -40.0
        navigation.map_origin_y = -42.2
        navigation.home = (0.0, -2.2, 0.4, 0.0)
        navigation.footprint_bounds = {
            "x_min": -10.0,
            "x_max": 10.0,
            "y_min": 0.0,
            "y_max": 36.0,
        }
        navigation._build_exploration_roi()
        stair_center = (
            0.5 * (
                navigation.stair_bounds["x_min"] +
                navigation.stair_bounds["x_max"]
            ),
            0.5 * (
                navigation.stair_bounds["y_min"] +
                navigation.stair_bounds["y_max"]
            ),
        )
        geometry = navigation._stair_geometry()
        stair_cell = navigation._cell(stair_center)
        entry_cell = navigation._cell((geometry[4], geometry[2]))

        self.assertFalse(
            navigation.exploration_roi[stair_cell[1], stair_cell[0]]
        )
        self.assertTrue(
            navigation.exploration_roi[entry_cell[1], entry_cell[0]]
        )

    def test_exploration_roi_excludes_unsupported_lobby_core_strips(self):
        navigation = _navigation_stub(size=200)
        navigation.map_origin_x = -40.0
        navigation.map_origin_y = -42.2
        navigation.home = (0.0, -2.2, 0.4, 0.0)
        navigation.footprint_bounds = {
            "x_min": -10.0,
            "x_max": 10.0,
            "y_min": 0.0,
            "y_max": 36.0,
        }
        navigation.lobby_bounds = {
            "x_min": -10.0,
            "x_max": 10.0,
            "y_min": 0.0,
            "y_max": 7.85,
        }
        navigation.elevator_bounds = {
            "x_min": 1.65,
            "x_max": 4.05,
            "y_min": 1.25,
            "y_max": 3.95,
        }

        navigation._build_exploration_roi()

        stair_void = navigation._cell((-3.25, 7.50))
        elevator_void = navigation._cell((2.85, 4.50))
        central_slab = navigation._cell((0.0, 4.50))
        self.assertFalse(
            navigation.exploration_roi[stair_void[1], stair_void[0]]
        )
        self.assertFalse(
            navigation.exploration_roi[elevator_void[1], elevator_void[0]]
        )
        self.assertTrue(
            navigation.exploration_roi[central_slab[1], central_slab[0]]
        )

    def test_roi_boundary_is_not_reported_as_frontier(self):
        navigation = _navigation_stub(size=40)
        navigation.exploration_roi.fill(False)
        navigation.exploration_roi[5:30, 8:13] = True
        belief = np.full((40, 40), UNKNOWN, dtype=np.uint8)
        belief[5:15, 8:13] = FREE
        frontier_cells = np.argwhere(
            navigation._exploration_frontier_mask(belief)
        )
        self.assertGreater(len(frontier_cells), 0)
        self.assertEqual(set(frontier_cells[:, 0].tolist()), {14})

    def test_main_entrance_restores_only_observed_free_cells(self):
        navigation = _navigation_stub(size=21)
        navigation.map_origin_x = -4.0
        navigation.map_origin_y = -4.0
        navigation.footprint_bounds = {
            "x_min": -10.0,
            "x_max": 10.0,
            "y_min": 0.0,
            "y_max": 36.0,
        }
        belief = np.full((21, 21), UNKNOWN, dtype=np.uint8)
        belief[8:13, 10] = FREE
        belief[10, 9] = OCCUPIED
        belief[10, 11] = OCCUPIED

        traversable = navigation._known_free_mask(belief)

        self.assertTrue(traversable[10, 10])
        self.assertFalse(traversable[10, 9])
        self.assertFalse(traversable[10, 11])
        self.assertFalse(traversable[9, 9])

    def test_observed_doorway_remains_connected_after_inflation(self):
        navigation = _navigation_stub(size=21)
        belief = np.full((21, 21), UNKNOWN, dtype=np.uint8)
        belief[7:14, 4:17] = FREE
        belief[7:10, 10] = OCCUPIED
        belief[11:14, 10] = OCCUPIED

        traversable = navigation._known_free_mask(belief)

        self.assertTrue(traversable[10, 9])
        self.assertTrue(traversable[10, 10])
        self.assertTrue(traversable[10, 11])
        self.assertFalse(traversable[9, 10])
        self.assertFalse(traversable[6, 10])

    def test_waypoints_drop_start_cell_and_preserve_goal(self):
        navigation = _navigation_stub(size=40)
        waypoints = navigation._cells_to_waypoints(
            [(10, 10), (11, 10), (12, 10)]
        )
        self.assertNotIn((4.0, 4.0), waypoints)
        self.assertAlmostEqual(waypoints[-1][0], 4.8)
        self.assertAlmostEqual(waypoints[-1][1], 4.0)

    def test_waypoints_preserve_grid_path_corners(self):
        navigation = _navigation_stub(size=40)
        waypoints = navigation._cells_to_waypoints(
            [(10, 10), (11, 10), (12, 10), (12, 11), (12, 12)]
        )

        self.assertTrue(any(
            abs(point[0] - 4.8) < 1e-6 and abs(point[1] - 4.0) < 1e-6
            for point in waypoints
        ))
        self.assertEqual(len(waypoints), 3)
        self.assertAlmostEqual(waypoints[-1][0], 4.8)
        self.assertAlmostEqual(waypoints[-1][1], 4.8)

    def test_verifier_coverage_uses_occupancy_grid_bounds(self):
        values = np.full((10, 10), -1, dtype=np.int8)
        values[:5, :] = 0
        position = SimpleNamespace(x=0.0, y=0.0)
        origin = SimpleNamespace(position=position)
        info = SimpleNamespace(width=10, height=10, resolution=0.4, origin=origin)
        message = SimpleNamespace(data=values.reshape(-1).tolist(), info=info)
        bounds = {"x_min": 0.0, "x_max": 3.6, "y_min": 0.0, "y_max": 3.6}
        self.assertAlmostEqual(
            FullMapVerifier._coverage_in_bounds(message, bounds), 0.5
        )


if __name__ == "__main__":
    unittest.main()
