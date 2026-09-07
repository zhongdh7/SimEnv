#!/usr/bin/env python3

import json
import math
import os
import sys
import tempfile
import unittest


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "tools"))

from evaluate_hazard_run import evaluate
from hazard_perception_v0.evaluation import maximum_cardinality_min_distance_matching


class HazardEvaluationTest(unittest.TestCase):
    def _evaluate(self, predictions, truth, threshold=0.75):
        with tempfile.TemporaryDirectory() as directory:
            prediction_path = os.path.join(directory, "hazards.json")
            truth_path = os.path.join(directory, "danger_truth.json")
            with open(prediction_path, "w", encoding="utf-8") as output:
                json.dump({"frame_id": "odom", "hazards": predictions}, output)
            with open(truth_path, "w", encoding="utf-8") as output:
                json.dump({"danger_sources": truth}, output)
            return evaluate(prediction_path, truth_path, threshold=threshold)

    def test_perfect_matching(self):
        report = self._evaluate(
            [{"id": 10, "x": 0, "y": 0, "z": 0,
              "confidence": 0.9, "localization_source": "rgbd"},
             {"id": 11, "x": 2, "y": 0, "z": 0,
              "confidence": 0.8, "localization_source": "fused"}],
            [{"id": 20, "position": [0, 0, 0]},
             {"id": 21, "position": [2, 0, 0]}],
            threshold=0.01,
        )
        self.assertEqual((report["TP"], report["FP"], report["FN"]), (2, 0, 0))
        self.assertEqual(report["detection_rate"], 1.0)
        self.assertEqual(report["precision"], 1.0)
        self.assertEqual(report["false_alarm_ratio"], 0.0)
        self.assertEqual(report["f1"], 1.0)
        self.assertEqual(len(report["matches"]), 2)

    def test_false_positive(self):
        report = self._evaluate(
            [{"id": 1, "x": 0, "y": 0, "z": 0},
             {"id": 2, "x": 5, "y": 0, "z": 0}],
            [{"id": 3, "position": [0, 0, 0]}], threshold=0.1,
        )
        self.assertEqual((report["TP"], report["FP"], report["FN"]), (1, 1, 0))
        self.assertAlmostEqual(report["false_alarm_ratio"], 0.5)
        self.assertEqual(report["false_positives"][0]["prediction_id"], 2)

    def test_false_negative(self):
        report = self._evaluate(
            [{"id": 1, "x": 0, "y": 0, "z": 0}],
            [{"id": 3, "position": [0, 0, 0]},
             {"id": 4, "position": [3, 0, 0]}], threshold=0.1,
        )
        self.assertEqual((report["TP"], report["FP"], report["FN"]), (1, 0, 1))
        self.assertAlmostEqual(report["miss_rate"], 0.5)
        self.assertEqual(report["false_negatives"][0]["truth_id"], 4)

    def test_threshold_is_inclusive(self):
        report = self._evaluate(
            [{"x": 0, "y": 0, "z": 0}],
            [{"position": [0.75, 0, 0]}], threshold=0.75,
        )
        self.assertEqual(report["TP"], 1)
        self.assertAlmostEqual(report["matches"][0]["error_m"], 0.75)

    def test_matching_maximizes_tp_before_distance(self):
        # Old full-assignment min-cost matching chose p0->t0 (0.0) and
        # p1->t1 (0.762), then dropped the latter, leaving only one TP.
        # The cross assignment has two threshold-valid pairs and must win.
        predictions = [(-1.0, -1.0, 0.0), (-1.0, -0.7, 0.0)]
        truth = [(-1.0, -1.0, 0.0), (-0.3, -1.0, 0.0)]
        matches = maximum_cardinality_min_distance_matching(predictions, truth, 0.75)
        self.assertEqual(len(matches), 2)
        self.assertEqual(
            [(item["prediction_index"], item["truth_index"]) for item in matches],
            [(0, 1), (1, 0)],
        )

    def test_start_frame_rotation_translation_and_error_statistics(self):
        with tempfile.TemporaryDirectory() as directory:
            prediction_path = os.path.join(directory, "hazards.json")
            truth_path = os.path.join(directory, "truth.json")
            output_path = os.path.join(directory, "evaluation.json")
            with open(prediction_path, "w", encoding="utf-8") as output:
                json.dump({
                    "frame_id": "start_frame",
                    "start_frame_transform": {
                        "parent_frame": "odom", "child_frame": "start_frame",
                        "translation": {"x": 10, "y": -2, "z": 0.5},
                        "quaternion_xyzw": {
                            "x": 0, "y": 0,
                            "z": math.sin(math.pi / 4),
                            "w": math.cos(math.pi / 4),
                        },
                    },
                    "hazards": [{"id": 5, "x": 1, "y": 0, "z": 0}],
                }, output)
            with open(truth_path, "w", encoding="utf-8") as output:
                json.dump({"danger_sources": [
                    {"id": 8, "position": [10, -1, 0.5]}
                ]}, output)
            report = evaluate(prediction_path, truth_path, output_path, 1.0e-6)
            self.assertEqual(report["truth_frame"], "odom")
            self.assertEqual(report["TP"], 1)
            self.assertAlmostEqual(report["mean_error_m"], 0.0, places=8)
            self.assertAlmostEqual(report["median_error_m"], 0.0, places=8)
            self.assertAlmostEqual(report["p95_error_m"], 0.0, places=8)
            self.assertTrue(os.path.isfile(output_path))

    def test_zero_predictions_uses_documented_false_alarm_ratio(self):
        report = self._evaluate([], [{"position": [0, 0, 0]}])
        self.assertEqual(report["prediction_count"], 0)
        self.assertEqual(report["false_alarm_ratio"], 0.0)


if __name__ == "__main__":
    unittest.main()
