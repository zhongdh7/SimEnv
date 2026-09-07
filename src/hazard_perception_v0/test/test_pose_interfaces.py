#!/usr/bin/env python3

import math
import unittest

from hazard_perception_v0.pose_interfaces import (
    normalize_odometry_data,
    normalize_quaternion,
    orientation_only_status,
    relative_orientation,
)


class PoseInterfacesTest(unittest.TestCase):
    def test_odometry_helper_preserves_pose_and_applies_frames(self):
        result = normalize_odometry_data(
            (1.0, -2.0, 0.3), (0.0, 0.0, 2.0, 2.0),
            "camera_init", "body", parent_override="odom", base_override="base",
        )
        self.assertEqual(result["position"], (1.0, -2.0, 0.3))
        self.assertEqual(result["parent_frame"], "odom")
        self.assertEqual(result["child_frame"], "base")
        self.assertAlmostEqual(sum(value * value for value in result["quaternion_xyzw"]), 1.0)

    def test_odometry_helper_rejects_nan_and_inf(self):
        with self.assertRaises(ValueError):
            normalize_odometry_data(
                (float("nan"), 0, 0), (0, 0, 0, 1), "odom", "base"
            )
        with self.assertRaises(ValueError):
            normalize_odometry_data(
                (0, 0, 0), (0, 0, float("inf"), 1), "odom", "base"
            )

    def test_quaternion_normalization(self):
        self.assertEqual(normalize_quaternion((0, 0, 0, 2)), (0, 0, 0, 1))
        with self.assertRaises(ValueError):
            normalize_quaternion((0, 0, 0, 0))

    def test_imu_relative_orientation_is_ninety_degrees(self):
        q0 = (0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8))
        current = (0.0, 0.0, math.sin(3 * math.pi / 8), math.cos(3 * math.pi / 8))
        relative = relative_orientation(q0, current)
        self.assertAlmostEqual(relative[0], 0.0, places=7)
        self.assertAlmostEqual(relative[1], 0.0, places=7)
        self.assertAlmostEqual(relative[2], math.sin(math.pi / 4), places=7)
        self.assertAlmostEqual(relative[3], math.cos(math.pi / 4), places=7)

    def test_imu_contract_never_claims_translation_or_global_position(self):
        status = orientation_only_status(True)
        self.assertTrue(status["orientation_valid"])
        self.assertFalse(status["translation_valid"])
        self.assertFalse(status["global_position_valid"])
        self.assertNotIn("x", status)
        self.assertNotIn("position", status)


if __name__ == "__main__":
    unittest.main()
