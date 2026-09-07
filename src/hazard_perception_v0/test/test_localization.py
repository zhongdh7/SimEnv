#!/usr/bin/env python3

import unittest

import numpy as np

from hazard_perception_v0.localization import (
    CameraIntrinsics,
    backproject_pixel,
    camera_models_aligned,
    depth_to_metres,
    fit_sphere_robust,
    monocular_sphere_position,
    inverse_transform_point_xyzw,
    optical_to_sensor_body,
    plane_fit_residual,
    rasterize_point_cloud_depth,
    robust_depth_from_detection,
    validate_sphere_geometry,
    transform_point_xyzw,
    sensor_body_to_optical,
)


class LocalizationTest(unittest.TestCase):
    def setUp(self):
        self.intrinsics = CameraIntrinsics(640, 480, 500.0, 500.0, 320.0, 240.0)

    def test_rgbd_coordinate_contract_round_trip(self):
        # A non-trivial yaw/pitch/roll transform catches axis swaps, reversed
        # TF direction, translation-only implementations and double transforms.
        quaternion = np.asarray([0.12, -0.21, 0.31, 0.91], dtype=np.float64)
        quaternion /= np.linalg.norm(quaternion)
        translation = np.asarray([1.7, -2.4, 0.8], dtype=np.float64)
        world = np.asarray([2.2, -1.1, 1.35], dtype=np.float64)
        body = inverse_transform_point_xyzw(world, translation, quaternion)
        optical = sensor_body_to_optical(body)
        recovered = transform_point_xyzw(
            optical_to_sensor_body(optical), translation, quaternion
        )
        np.testing.assert_allclose(recovered, world, atol=1.0e-12)

    def test_backprojects_principal_point_and_offset(self):
        np.testing.assert_allclose(
            backproject_pixel(320.0, 240.0, 2.0, self.intrinsics),
            [0.0, 0.0, 2.0],
        )
        np.testing.assert_allclose(
            backproject_pixel(420.0, 190.0, 2.0, self.intrinsics),
            [0.4, -0.2, 2.0],
        )

    def test_robust_depth_filters_invalid_and_outlier_values(self):
        depth = np.full((20, 20), np.nan, dtype=np.float32)
        mask = np.zeros((20, 20), dtype=np.uint8)
        mask[5:15, 5:15] = 255
        depth[5:15, 5:15] = 2.0
        depth[5, 5] = 0.0
        depth[6, 6] = np.inf
        depth[7, 7] = 50.0
        depth[8, 8] = 3.5

        estimate = robust_depth_from_detection(
            depth, mask, (4, 4, 12, 12), 0.4, 8.0, 20
        )
        self.assertIsNotNone(estimate)
        self.assertAlmostEqual(estimate.depth, 2.0)
        self.assertGreaterEqual(estimate.valid_count, 96)

    def test_rejects_too_few_valid_depth_pixels(self):
        depth = np.zeros((10, 10), dtype=np.float32)
        mask = np.ones((10, 10), dtype=np.uint8) * 255
        depth[2, 2] = 1.0
        self.assertIsNone(
            robust_depth_from_detection(depth, mask, (0, 0, 10, 10), 0.4, 8.0, 2)
        )

    def test_depth_encoding_and_alignment(self):
        raw = np.asarray([[1000, 2500]], dtype=np.uint16)
        np.testing.assert_allclose(depth_to_metres(raw, "16UC1"), [[1.0, 2.5]])
        self.assertTrue(camera_models_aligned(self.intrinsics, self.intrinsics))
        shifted = CameraIntrinsics(640, 480, 500.0, 500.0, 321.0, 240.0)
        self.assertFalse(camera_models_aligned(self.intrinsics, shifted))

    def test_rasterizes_nearest_point_for_unaligned_depth_fallback(self):
        model = CameraIntrinsics(5, 5, 2.0, 2.0, 2.0, 2.0)
        points = np.asarray([[0.0, 0.0, 2.0], [0.0, 0.0, 1.0], [5.0, 0.0, 1.0]])
        registered = rasterize_point_cloud_depth(points, model, 0.4, 8.0)
        self.assertAlmostEqual(registered[2, 2], 1.0)
        self.assertTrue(np.isnan(registered[0, 0]))

    @staticmethod
    def _visible_sphere_points(center, radius, noise=0.001):
        rng = np.random.RandomState(7)
        xy = rng.uniform(-0.13, 0.13, (3000, 2))
        xy = xy[np.sum(xy * xy, axis=1) < radius * radius][:1000]
        z = center[2] - np.sqrt(radius * radius - np.sum(xy * xy, axis=1))
        points = np.column_stack((center[0] + xy[:, 0], center[1] + xy[:, 1], z))
        return points + rng.normal(0.0, noise, points.shape)

    def test_known_and_free_radius_sphere_fits_recover_center(self):
        center = np.asarray([0.2, -0.1, 2.0])
        points = self._visible_sphere_points(center, 0.15)
        for mode in ("known_radius", "free_radius"):
            fit = fit_sphere_robust(
                points, mode=mode, known_radius=0.15,
                min_radius=0.10, max_radius=0.22,
            )
            self.assertIsNotNone(fit)
            np.testing.assert_allclose(fit.center, center, atol=0.004)
            self.assertAlmostEqual(fit.radius, 0.15, delta=0.004)
            validation = validate_sphere_geometry(
                fit, plane_fit_residual(points), 0.8,
                0.012, 0.65, 0.003, 0.20, 0.65,
            )
            self.assertTrue(validation.accepted, validation.reason)

    def test_planar_red_patch_fails_sphere_geometry(self):
        rng = np.random.RandomState(11)
        x, y = np.meshgrid(np.linspace(-0.15, 0.15, 35), np.linspace(-0.15, 0.15, 35))
        points = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, 2.0)))
        points += rng.normal(0.0, 0.0005, points.shape)
        fit = fit_sphere_robust(points, mode="known_radius", known_radius=0.15)
        validation = validate_sphere_geometry(
            fit, plane_fit_residual(points), 0.0,
            0.012, 0.65, 0.003, 0.20, 0.65,
        )
        self.assertFalse(validation.accepted)
        self.assertIn("too_planar", validation.reason)

    def test_monocular_known_size_estimate(self):
        position = monocular_sphere_position(320.0, 240.0, 15.0, 0.15, self.intrinsics)
        self.assertIsNotNone(position)
        self.assertAlmostEqual(position[0], 0.0)
        self.assertAlmostEqual(position[1], 0.0)
        self.assertAlmostEqual(position[2], 5.0, delta=0.01)


if __name__ == "__main__":
    unittest.main()
