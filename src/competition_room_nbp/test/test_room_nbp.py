#!/usr/bin/env python3

import json
import math
import os
import sys
import tempfile
import unittest

import numpy as np

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from competition_room_nbp.hazard_nbv import HazardNBVSelector
from competition_room_nbp.planner import RoomNBPPlanner


class _Backend(object):
    ready = True
    reason = "ready"

    def infer(self, model_input):
        assert model_input.shape == (1, 5, 256, 256)
        values = np.zeros((8, 64, 64), dtype=np.float32)
        values[3, 32, 30] = 7.5
        return values, 12.0, 10.0

    def close(self):
        pass


class RoomNBPPlannerTest(unittest.TestCase):
    def test_hazard_nbv_prefers_short_high_value_view_and_task_yaw(self):
        selector = HazardNBVSelector(min_gain_cells=4, max_path_m=6.0)
        candidates = [
            {"cell": (1, 1), "world": (1.0, 0.0), "path": [(1.0, 0.0)],
             "path_m": 1.2, "hazard_visibility": [0, 0, 71, 0, 0, 0, 0, 0],
             "camera_visibility": [0] * 8},
            {"cell": (8, 8), "world": (8.0, 0.0), "path": [(8.0, 0.0)],
             "path_m": 11.2, "hazard_visibility": [100] * 8,
             "camera_visibility": [0] * 8},
            {"cell": (3, 3), "world": (3.0, 0.0), "path": [(3.0, 0.0)],
             "path_m": 2.4, "hazard_visibility": [0, 0, 0, 0, 91, 0, 0, 0],
             "camera_visibility": [0] * 8},
        ]
        selected = selector.select((0.0, 0.0, 0.3, 0.0), candidates)
        self.assertEqual(selected["cell"], (1, 1))
        self.assertEqual(selected["yaw_channel"], 2)
        self.assertEqual(selected["strategy"], "hazard_nbv")

    def test_hazard_nbv_rejects_low_gain_and_long_paths(self):
        selector = HazardNBVSelector(min_gain_cells=5, max_path_m=3.0)
        candidates = [
            {"path_m": 4.0, "hazard_visibility": [100] * 8},
            {"path_m": 1.0, "hazard_visibility": [4] * 8},
        ]
        self.assertIsNone(selector.select((0.0, 0.0, 0.0, 0.0), candidates))

    def planner(self, log_path=""):
        planner = RoomNBPPlanner(
            False, "", "", "", view_range=12.8, log_path=log_path
        )
        planner.enabled = True
        planner.backend = _Backend()
        return planner

    def test_projection_has_four_height_layers_and_trajectory(self):
        planner = self.planner()
        pose = (0.0, 0.0, 0.3, 0.0)
        planner.begin_room(4, {
            "x_min": -4.0, "x_max": 4.0,
            "y_min": -4.0, "y_max": 4.0,
        }, 0.0, pose)
        planner.observe_scan([
            (1.0, 0.0, 0.0), (1.0, 0.2, 0.7),
            (1.0, 0.4, 1.4), (1.0, 0.6, 2.1),
        ], pose)
        planner.observe_pose((0.3, 0.0, 0.3, 0.0))

        model_input, points, poses = planner._project(pose)

        self.assertEqual(model_input.shape, (1, 5, 256, 256))
        self.assertEqual(points, 4)
        self.assertEqual(poses, 2)
        for channel in range(5):
            self.assertGreater(model_input[0, channel].sum(), 0.0)

    def test_select_preserves_prevalidated_path_and_decodes_yaw(self):
        planner = self.planner()
        pose = (0.0, 0.0, 0.3, 0.0)
        planner.begin_room(2, {
            "x_min": -5.0, "x_max": 5.0,
            "y_min": -5.0, "y_max": 5.0,
        }, 0.0, pose)
        # row=32, col=30 at range 12.8 corresponds to x=+0.8,y=0.
        candidate = {
            "cell": (10, 8), "world": (0.8, 0.0),
            "path": [(0.4, 0.0), (0.8, 0.0)], "path_m": 0.8,
        }

        selected = planner.select(pose, [candidate])

        self.assertEqual(selected["cell"], candidate["cell"])
        self.assertEqual(selected["path"], candidate["path"])
        self.assertAlmostEqual(selected["yaw"], 3.0 * math.pi / 4.0)
        self.assertAlmostEqual(selected["value"], 7.5)

    def test_finish_log_contains_room_duration_and_target_path(self):
        with tempfile.TemporaryDirectory() as directory:
            log_path = os.path.join(directory, "events.jsonl")
            planner = self.planner(log_path)
            pose = (0.0, 0.0, 0.3, 0.0)
            planner.begin_room(1, {
                "x_min": -5.0, "x_max": 5.0,
                "y_min": -5.0, "y_max": 5.0,
            }, 0.0, pose)
            planner.select(pose, [{
                "cell": (1, 2), "world": (0.8, 0.0),
                "path": [(0.8, 0.0)], "path_m": 0.8,
            }])
            planner.finish_room("coverage_reached", 0.72, pose)
            with open(log_path, encoding="utf-8") as stream:
                events = [json.loads(line) for line in stream]
            self.assertEqual([event["event"] for event in events],
                             ["room_start", "inference", "target", "room_finish"])
            self.assertIn("path", events[2])
            self.assertGreaterEqual(events[-1]["room_duration_s"], 0.0)

    def test_entry_failure_can_be_logged_before_begin_room(self):
        with tempfile.TemporaryDirectory() as directory:
            log_path = os.path.join(directory, "events.jsonl")
            planner = self.planner(log_path)
            planner.log_event(
                "room_entry_failed", room_id=7, entry_status="deferred",
                room_search_status="not_started",
            )
            with open(log_path, encoding="utf-8") as stream:
                event = json.loads(stream.readline())
            self.assertEqual(event["room_id"], 7)
            self.assertEqual(event["entry_status"], "deferred")
            self.assertEqual(event["room_search_status"], "not_started")

    def test_camera_and_path_terms_can_override_raw_yaw_argmax(self):
        planner = self.planner()
        pose = (0.0, 0.0, 0.3, 0.0)
        planner.begin_room(3, {
            "x_min": -5.0, "x_max": 5.0,
            "y_min": -5.0, "y_max": 5.0,
        }, 0.0, pose)
        candidate = {
            "cell": (1, 2), "world": (0.8, 0.0),
            "path": [(0.8, 0.0)], "path_m": 0.8,
            "camera_visibility": [0.0, 0.0, 0.0, 0.0, 12.0, 0.0, 0.0, 0.0],
        }
        selected = planner.select(
            pose, [candidate], camera_weight=2.0, path_weight=0.0
        )
        self.assertEqual(selected["yaw_channel"], 4)
        self.assertGreater(selected["camera_new_visibility"], 0.0)
        self.assertGreater(selected["score"], 0.0)

    def test_hazard_visibility_term_prefers_unobserved_view(self):
        planner = self.planner()
        pose = (0.0, 0.0, 0.3, 0.0)
        planner.begin_room(6, {
            "x_min": -5.0, "x_max": 5.0,
            "y_min": -5.0, "y_max": 5.0,
        }, 0.0, pose)
        common = {
            "cell": (10, 8), "world": (0.8, 0.0),
            "path": [(0.8, 0.0)], "path_m": 0.8,
        }
        common["hazard_visibility"] = [12.0] + [0.0] * 7
        selected = planner.select(
            pose, [common], hazard_weight=2.0
        )
        self.assertEqual(selected["yaw_channel"], 0)
        self.assertGreater(selected["hazard_new_visibility"], 0.0)
        self.assertGreater(selected["hazard_visibility_norm"], 0.0)


if __name__ == "__main__":
    unittest.main()
