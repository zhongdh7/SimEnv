#!/usr/bin/env python3

import unittest

import numpy as np

from hazard_perception.livox_fusion import choose_fused_position, estimate_livox_position
from hazard_perception.localization import CameraIntrinsics


class LivoxFusionTest(unittest.TestCase):
    def test_extracts_cluster_inside_red_detection(self):
        intrinsics = CameraIntrinsics(100, 80, 50.0, 50.0, 50.0, 40.0)
        mask = np.zeros((80, 100), dtype=np.uint8)
        mask[35:46, 45:56] = 255
        cluster = np.asarray(
            [[-0.04, 0.0, 2.0], [0.0, 0.02, 2.02], [0.04, -0.02, 1.98], [0.02, 0.0, 2.01]]
        )
        background = np.asarray([[2.0, 0.0, 2.0], [0.0, 1.5, 2.0]])
        result = estimate_livox_position(
            np.vstack((cluster, background)), mask, (44, 34, 13, 13), intrinsics,
            0.2, 10.0, 3, 0.2,
        )
        self.assertIsNotNone(result)
        self.assertEqual(result.point_count, 4)
        self.assertEqual(result.points.shape, (4, 3))
        np.testing.assert_allclose(result.position, [0.01, 0.0, 2.005], atol=0.03)

    def test_fusion_selection_and_fallbacks(self):
        rgbd = np.asarray([1.0, 2.0, 0.5])
        livox = np.asarray([1.2, 2.0, 0.5])
        fused = choose_fused_position(rgbd, livox, 0.5, 3.0, 1.0)
        self.assertEqual(fused.source, "fused")
        np.testing.assert_allclose(fused.position, [1.05, 2.0, 0.5])
        self.assertEqual(choose_fused_position(rgbd, None, 0.5, 1.0, 1.0).source, "rgbd")
        self.assertEqual(choose_fused_position(None, livox, 0.5, 1.0, 1.0).source, "livox")
        self.assertIsNone(choose_fused_position(None, None, 0.5, 1.0, 1.0))
        rejected = choose_fused_position(rgbd, rgbd + 2.0, 0.5, 1.0, 1.0)
        self.assertEqual(rejected.source, "rgbd")


if __name__ == "__main__":
    unittest.main()
