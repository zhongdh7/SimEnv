#!/usr/bin/env python3

import json
import os
import sys
import tempfile
import unittest
import math

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"
))
from evaluate_floor0 import evaluate


class EvaluateFloor0Test(unittest.TestCase):
    def test_transforms_start_frame_and_filters_floor0_truth(self):
        prediction = {
            "frame_id": "start_frame",
            "start_frame_transform": {
                "translation": {"x": 1.0, "y": 2.0, "z": 0.0},
                "quaternion_xyzw": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                "parent_frame": "odom", "child_frame": "start_frame",
            },
            "hazards": [{"id": 2, "x": 1.0, "y": 2.0, "z": 0.15}],
        }
        truth = {"danger_sources": [
            {"id": 4, "floor_index": 0, "position": [2.0, 4.0, 0.15]},
            {"id": 5, "floor_index": 1, "position": [2.0, 4.0, 2.75]},
        ]}
        with tempfile.TemporaryDirectory() as directory:
            prediction_path = os.path.join(directory, "prediction.json")
            truth_path = os.path.join(directory, "truth.json")
            with open(prediction_path, "w") as output:
                json.dump(prediction, output)
            with open(truth_path, "w") as output:
                json.dump(truth, output)
            report = evaluate(prediction_path, truth_path, threshold=0.01)
        self.assertEqual(report["prediction_frame"], "start_frame")
        self.assertEqual(report["GT_frame"], "world")
        self.assertEqual(report["gt_count"], 1)
        self.assertEqual(report["TP"], 1)
        self.assertAlmostEqual(report["matches"][0]["error_m"], 0.0)

    def test_evaluator_applies_rotation_and_translation_once(self):
        # T_(world,start) is a 90-degree yaw plus translation.  A prediction
        # at (1,0,0.15) in start_frame therefore lands at (1,3,0.15) world.
        prediction = {
            "frame_id": "start_frame",
            "start_frame_transform": {
                "translation": {"x": 1.0, "y": 2.0, "z": 0.0},
                "quaternion_xyzw": {
                    "x": 0.0, "y": 0.0,
                    "z": math.sin(math.pi / 4.0),
                    "w": math.cos(math.pi / 4.0),
                },
                "parent_frame": "world", "child_frame": "start_frame",
            },
            "hazards": [{"id": 9, "x": 1.0, "y": 0.0, "z": 0.15}],
        }
        truth = {"danger_sources": [
            {"id": 10, "floor_index": 0, "position": [1.0, 3.0, 0.15]},
        ]}
        with tempfile.TemporaryDirectory() as directory:
            prediction_path = os.path.join(directory, "prediction.json")
            truth_path = os.path.join(directory, "truth.json")
            with open(prediction_path, "w") as output:
                json.dump(prediction, output)
            with open(truth_path, "w") as output:
                json.dump(truth, output)
            report = evaluate(prediction_path, truth_path, threshold=1.0e-6)
        self.assertEqual(report["TP"], 1)
        self.assertAlmostEqual(report["matches"][0]["error_m"], 0.0)


if __name__ == "__main__":
    unittest.main()
