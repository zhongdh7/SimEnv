#!/usr/bin/env python3

import math
import os
import sys
import unittest


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "scripts"))

from hazard_finalize_cli import transform_hazards


class HazardFinalizeCliTest(unittest.TestCase):
    def test_transforms_finalized_start_frame_records(self):
        payload = {
            "frame_id": "start_frame",
            "start_frame_transform": {
                "parent_frame": "odom", "child_frame": "start_frame",
                "translation": {"x": 1, "y": 2, "z": 0.5},
                "quaternion_xyzw": {
                    "x": 0, "y": 0,
                    "z": math.sin(math.pi / 4),
                    "w": math.cos(math.pi / 4),
                },
            },
            "hazards": [{
                "id": 3, "x": 1, "y": 0, "z": 0,
                "confidence": 0.8, "observation_count": 4,
                "localization_source": "rgbd",
            }],
        }
        transformed = transform_hazards(payload, "odom")
        self.assertAlmostEqual(transformed[0]["x"], 1.0, places=7)
        self.assertAlmostEqual(transformed[0]["y"], 3.0, places=7)
        self.assertAlmostEqual(transformed[0]["z"], 0.5, places=7)
        self.assertEqual(transformed[0]["observation_count"], 4)

    def test_rejects_wrong_map_frame(self):
        payload = {
            "frame_id": "start_frame",
            "start_frame_transform": {
                "parent_frame": "odom", "child_frame": "start_frame",
                "translation": {"x": 0, "y": 0, "z": 0},
                "quaternion_xyzw": {"x": 0, "y": 0, "z": 0, "w": 1},
            },
            "hazards": [],
        }
        with self.assertRaises(ValueError):
            transform_hazards(payload, "map")


if __name__ == "__main__":
    unittest.main()
