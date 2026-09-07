#!/usr/bin/env python3

import json
import os
import tempfile
import unittest

import numpy as np

from hazard_perception.result_manager import ResultManager
from hazard_perception.tracker import HazardTrack


class ResultManagerTest(unittest.TestCase):
    def test_saves_only_confirmed_tracks_as_json_and_csv(self):
        confirmed = HazardTrack(
            7, np.asarray([1.0, -2.0, 0.4]), 4, 3.2, 1.0, 2.0, True, "fused", {"fused": 4}
        )
        candidate = HazardTrack(
            8, np.zeros(3), 1, 0.5, 1.0, 1.0, False, "rgbd", {"rgbd": 1}
        )
        with tempfile.TemporaryDirectory() as directory:
            manager = ResultManager(directory, save_csv=True, run_id="unit")
            json_path, csv_path = manager.save([confirmed, candidate])
            self.assertTrue(os.path.isfile(csv_path))
            with open(json_path, encoding="utf-8") as source:
                payload = json.load(source)
            self.assertEqual(payload["hazard_count"], 1)
            self.assertEqual(payload["frame_id"], "start_frame")
            self.assertIsNone(payload["start_frame_transform"])
            self.assertEqual(payload["hazards"][0]["id"], 7)
            self.assertEqual(payload["hazards"][0]["x"], 1.0)
            self.assertEqual(payload["hazards"][0]["y"], -2.0)
            self.assertEqual(payload["hazards"][0]["z"], 0.4)
            self.assertEqual(payload["hazards"][0]["localization_source"], "fused")
            second_json_path, _ = manager.save([confirmed])
            self.assertNotEqual(json_path, second_json_path)

    def test_saves_start_frame_transform_for_offline_evaluation(self):
        class _Vector(object):
            pass
        transform = _Vector()
        transform.header = _Vector()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "start_frame"
        transform.transform = _Vector()
        transform.transform.translation = _Vector()
        transform.transform.translation.x = 1.0
        transform.transform.translation.y = 2.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation = _Vector()
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        confirmed = HazardTrack(
            1, np.asarray([0.0, 0.0, 0.0]), 4, 3.2, 1.0, 2.0, True, "rgbd", {"rgbd": 4}
        )
        with tempfile.TemporaryDirectory() as directory:
            manager = ResultManager(directory, run_id="frame")
            manager.set_start_frame_transform(transform)
            path, _ = manager.save([confirmed])
            with open(path, encoding="utf-8") as source:
                payload = json.load(source)
            self.assertEqual(payload["start_frame_transform"]["parent_frame"], "odom")
            self.assertEqual(payload["start_frame_transform"]["translation"]["x"], 1.0)

    def test_saves_track_diagnostics_without_changing_core_fields(self):
        confirmed = HazardTrack(
            3, np.asarray([1.0, 2.0, 0.15]), 3, 2.4, 1.0, 2.0, True, "rgbd", {"rgbd": 3}
        )
        with tempfile.TemporaryDirectory() as directory:
            manager = ResultManager(directory, run_id="diagnostics")
            path, _ = manager.save([confirmed], {
                3: {"bbox": [1, 2, 3, 4], "raw_camera_xyz": [0.1, 0.2, 1.0]}
            })
            with open(path, encoding="utf-8") as source:
                payload = json.load(source)
            self.assertEqual(payload["hazards"][0]["diagnostics"]["bbox"], [1, 2, 3, 4])


if __name__ == "__main__":
    unittest.main()
