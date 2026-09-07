#!/usr/bin/env python3

import importlib.util
import os
import unittest

import numpy as np


MODULE_PATH = os.path.join(os.path.dirname(__file__), "hazard_validation.py")
SPEC = importlib.util.spec_from_file_location("hazard_validation", MODULE_PATH)
VALIDATION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATION)


class ValidationMetricsTest(unittest.TestCase):
    def test_sample_metrics(self):
        metrics = VALIDATION.compute_sample_metrics(
            [[1.0, 2.0, 3.0], [1.2, 2.0, 3.0]], [1.0, 2.0, 3.0]
        )
        self.assertEqual(metrics["sample_count"], 2)
        self.assertAlmostEqual(metrics["mean_error_m"], 0.1)
        self.assertAlmostEqual(metrics["std_xyz_m"]["x"], 0.1)

    def test_empty_metrics(self):
        metrics = VALIDATION.compute_sample_metrics([], [0.0, 0.0, 0.0])
        self.assertEqual(metrics["sample_count"], 0)
        self.assertIsNone(metrics["mean_error_m"])

    def test_summary(self):
        cases = [
            {"expected_hazard": True, "success": True, "mean_error_m": 0.2,
             "std_norm_m": 0.01, "duplicate_target_count": 0,
             "false_positive_count": 0, "two_d_detected": True,
             "three_d_available": True, "source": "rgbd"},
            {"expected_hazard": True, "success": False, "mean_error_m": None,
             "std_norm_m": None, "duplicate_target_count": 1,
             "false_positive_count": 0, "two_d_detected": True,
             "three_d_available": False, "source": "2d_only"},
            {"expected_hazard": False, "success": False, "mean_error_m": None,
             "std_norm_m": None, "duplicate_target_count": 0,
             "false_positive_count": 2, "two_d_detected": True,
             "three_d_available": False, "source": "2d_only"},
        ]
        summary = VALIDATION.summarize_cases(cases)
        self.assertEqual(summary["detection_success_rate"], 0.5)
        self.assertEqual(summary["duplicate_target_count"], 1)
        self.assertEqual(summary["false_positive_case_rate"], 1.0)
        self.assertEqual(summary["false_positive_detection_count"], 2)
        self.assertEqual(summary["two_d_detection_success_rate"], 1.0)
        self.assertEqual(summary["three_d_data_availability_rate"], 0.5)
        self.assertEqual(summary["source_case_counts"]["rgbd"], 1)
        self.assertEqual(summary["source_case_counts"]["2d_only"], 2)
        self.assertEqual(summary["positive_source_case_counts"]["rgbd"], 1)
        self.assertEqual(summary["positive_source_case_counts"]["2d_only"], 1)


if __name__ == "__main__":
    unittest.main()
