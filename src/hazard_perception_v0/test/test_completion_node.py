#!/usr/bin/env python3

import json
import os
import sys
import tempfile
import unittest


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "scripts"))

from hazard_completion_node import HazardCompletionNode, transform_point


class HazardCompletionNodeTest(unittest.TestCase):
    def test_transform_point_uses_parent_from_persisted_start_frame(self):
        transform = {
            "parent_frame": "odom",
            "child_frame": "start_frame",
            "translation": {"x": 10.0, "y": -2.0, "z": 0.5},
            "quaternion_xyzw": {
                "x": 0.0, "y": 0.0,
                "z": 0.7071067811865476,
                "w": 0.7071067811865476,
            },
        }
        point = transform_point(transform, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(point[0], 10.0, places=6)
        self.assertAlmostEqual(point[1], -1.0, places=6)
        self.assertAlmostEqual(point[2], 0.5, places=6)

    def test_load_and_transform_preserves_confirmed_records(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "hazards.json")
            with open(path, "w", encoding="utf-8") as output:
                json.dump({
                    "frame_id": "start_frame",
                    "start_frame_transform": {
                        "parent_frame": "odom",
                        "child_frame": "start_frame",
                        "translation": {"x": 1.0, "y": 2.0, "z": 0.0},
                        "quaternion_xyzw": {
                            "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0,
                        },
                    },
                    "hazard_count": 1,
                    "hazards": [{
                        "id": 4, "x": 0.5, "y": -0.5, "z": 0.2,
                        "confirmed": True,
                    }],
                }, output)
            node = HazardCompletionNode.__new__(HazardCompletionNode)
            node.map_frame = "odom"
            payload, hazards = node._load_and_transform(path)
            self.assertEqual(payload["hazard_count"], 1)
            self.assertEqual(hazards[0]["id"], 4)
            self.assertAlmostEqual(hazards[0]["x"], 1.5)
            self.assertAlmostEqual(hazards[0]["y"], 1.5)
            self.assertEqual(hazards[0]["source_frame"], "start_frame")

    def test_load_and_transform_rejects_missing_alignment(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "hazards.json")
            with open(path, "w", encoding="utf-8") as output:
                json.dump({
                    "frame_id": "start_frame",
                    "hazards": [{"id": 1, "x": 0.0, "y": 0.0, "z": 0.0}],
                }, output)
            node = HazardCompletionNode.__new__(HazardCompletionNode)
            node.map_frame = "odom"
            with self.assertRaises(ValueError):
                node._load_and_transform(path)


if __name__ == "__main__":
    unittest.main()
