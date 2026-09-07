#!/usr/bin/env python3

import unittest

import cv2
import numpy as np

from hazard_perception.detector import DetectorConfig, RedSphereDetector


class RedSphereDetectorTest(unittest.TestCase):
    def setUp(self):
        self.detector = RedSphereDetector(
            DetectorConfig(
                blur_kernel_size=1,
                morphology_kernel_size=3,
                open_iterations=0,
                close_iterations=0,
                min_area=100.0,
                min_circularity=0.70,
            )
        )

    def test_detects_red_circle_and_rejects_elongated_region(self):
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.circle(image, (90, 120), 30, (0, 0, 255), thickness=-1)
        cv2.rectangle(image, (190, 105), (290, 125), (0, 0, 255), thickness=-1)

        mask, detections = self.detector.detect(image)

        self.assertEqual(mask.dtype, np.uint8)
        self.assertEqual(len(detections), 1)
        self.assertAlmostEqual(detections[0].center_x, 90.0, delta=1.0)
        self.assertAlmostEqual(detections[0].center_y, 120.0, delta=1.0)
        self.assertGreater(detections[0].circularity, 0.80)

    def test_rejects_small_red_circle_by_area(self):
        image = np.zeros((100, 100, 3), dtype=np.uint8)
        cv2.circle(image, (50, 50), 3, (0, 0, 255), thickness=-1)

        _, detections = self.detector.detect(image)

        self.assertEqual(detections, [])

    def test_rejects_invalid_even_kernel(self):
        with self.assertRaises(ValueError):
            RedSphereDetector(DetectorConfig(morphology_kernel_size=4))

    def test_accepts_border_circle_with_arc_fit(self):
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.circle(image, (321, 120), 28, (0, 0, 255), thickness=-1)

        _, detections = self.detector.detect(image)

        self.assertEqual(len(detections), 1)
        self.assertTrue(detections[0].touches_border)
        self.assertAlmostEqual(detections[0].center_x, 321.0, delta=3.0)
        self.assertAlmostEqual(detections[0].fitted_radius_px, 28.0, delta=3.0)
        self.assertLess(detections[0].edge_fit_residual, 0.10)


if __name__ == "__main__":
    unittest.main()
