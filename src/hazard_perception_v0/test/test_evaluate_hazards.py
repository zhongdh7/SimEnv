#!/usr/bin/env python3

import json
import os
import sys
import tempfile
import unittest


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "tools"))

from evaluate_hazards import evaluate


class EvaluateHazardsTest(unittest.TestCase):
    def test_all_floor_score_uses_persisted_sensor_frame_transform(self):
        with tempfile.TemporaryDirectory() as directory:
            prediction_path = os.path.join(directory, "predictions.json")
            truth_path = os.path.join(directory, "truth.json")
            with open(prediction_path, "w", encoding="utf-8") as output:
                json.dump({
                    "frame_id": "start_frame",
                    "start_frame_transform": {
                        "translation": {"x": 1.0, "y": 2.0, "z": 0.0},
                        "quaternion_xyzw": {
                            "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0,
                        },
                    },
                    "hazards": [{"x": 0.5, "y": -0.5, "z": 0.2}],
                }, output)
            with open(truth_path, "w", encoding="utf-8") as output:
                json.dump({
                    "danger_sources": [{"position": [1.5, 1.5, 0.2]}],
                }, output)
            report = evaluate(prediction_path, truth_path, threshold=0.01)
            self.assertTrue(report["passed"])
            self.assertEqual(report["TP"], 1)
            self.assertEqual(report["FP"], 0)
            self.assertEqual(report["FN"], 0)

    def test_extra_sensor_detection_fails_exact_completion_score(self):
        with tempfile.TemporaryDirectory() as directory:
            prediction_path = os.path.join(directory, "predictions.json")
            truth_path = os.path.join(directory, "truth.json")
            with open(prediction_path, "w", encoding="utf-8") as output:
                json.dump({
                    "frame_id": "odom",
                    "hazards": [
                        {"x": 0.0, "y": 0.0, "z": 0.0},
                        {"x": 5.0, "y": 0.0, "z": 0.0},
                    ],
                }, output)
            with open(truth_path, "w", encoding="utf-8") as output:
                json.dump({
                    "danger_sources": [{"position": [0.0, 0.0, 0.0]}],
                }, output)
            report = evaluate(prediction_path, truth_path, threshold=0.1)
            self.assertFalse(report["passed"])
            self.assertEqual(report["FP"], 1)


if __name__ == "__main__":
    unittest.main()
