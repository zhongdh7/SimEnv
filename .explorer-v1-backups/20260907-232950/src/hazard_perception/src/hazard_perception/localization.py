"""Testable RGB-D geometry, sphere fitting and robust depth helpers."""

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple

import numpy as np


def optical_to_sensor_body(point):
    """Convert ROS optical axes (x right, y down, z forward) to sensor body.

    SimEnv labels the RGB-D stream with the sensor body frame while the
    back-projected pixels are expressed in optical axes.  Keeping this
    conversion in one small, dependency-free function makes the production
    and offline coordinate contracts testable together.
    """
    point = np.asarray(point, dtype=np.float64)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError("point must be a finite XYZ vector")
    return np.asarray([point[2], -point[0], -point[1]], dtype=np.float64)


def sensor_body_to_optical(point):
    """Inverse of :func:`optical_to_sensor_body`."""
    point = np.asarray(point, dtype=np.float64)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError("point must be a finite XYZ vector")
    return np.asarray([-point[1], -point[2], point[0]], dtype=np.float64)


def rotate_xyzw(quaternion, point):
    """Apply an xyzw quaternion rotation to a 3-D point."""
    x, y, z, w = [float(value) for value in quaternion]
    px, py, pz = [float(value) for value in point]
    return np.asarray([
        (1 - 2 * (y * y + z * z)) * px + 2 * (x * y - z * w) * py
        + 2 * (x * z + y * w) * pz,
        2 * (x * y + z * w) * px + (1 - 2 * (x * x + z * z)) * py
        + 2 * (y * z - x * w) * pz,
        2 * (x * z - y * w) * px + 2 * (y * z + x * w) * py
        + (1 - 2 * (x * x + y * y)) * pz,
    ], dtype=np.float64)


def transform_point_xyzw(point, translation, quaternion):
    """Apply a parent<-child rigid transform to a point."""
    return rotate_xyzw(quaternion, point) + np.asarray(translation, dtype=np.float64)


def inverse_transform_point_xyzw(point, translation, quaternion):
    """Apply the inverse of a parent<-child rigid transform."""
    q = np.asarray([float(value) for value in quaternion], dtype=np.float64)
    return rotate_xyzw([-q[0], -q[1], -q[2], q[3]],
                       np.asarray(point, dtype=np.float64) - np.asarray(translation))


@dataclass(frozen=True)
class CameraIntrinsics:
    """Minimal pinhole model in ROS CameraInfo convention."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_camera_info(cls, message):
        return cls(
            width=int(message.width),
            height=int(message.height),
            fx=float(message.K[0]),
            fy=float(message.K[4]),
            cx=float(message.K[2]),
            cy=float(message.K[5]),
        )

    def validate(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Camera dimensions must be positive")
        if not np.isfinite((self.fx, self.fy, self.cx, self.cy)).all():
            raise ValueError("Camera intrinsics must be finite")
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("Camera focal lengths must be positive")


@dataclass(frozen=True)
class DepthEstimate:
    depth: float
    pixel_u: float
    pixel_v: float
    valid_count: int
    raw_valid_count: int


@dataclass(frozen=True)
class MaskedPointSet:
    """Valid masked RGB-D samples in the optical camera frame."""

    points: np.ndarray
    pixels: np.ndarray
    depths: np.ndarray
    raw_valid_count: int


@dataclass(frozen=True)
class SphereFitResult:
    """Robust sphere parameters and residual statistics."""

    center: np.ndarray
    radius: float
    residual_rms: float
    residual_median: float
    inlier_count: int
    point_count: int
    converged: bool
    mode: str


@dataclass(frozen=True)
class SphereGeometryResult:
    """Sphere-versus-plane structural validation result."""

    accepted: bool
    sphere: SphereFitResult
    plane_residual_rms: float
    radial_depth_correlation: float
    confidence: float
    reason: str


def camera_models_aligned(
    rgb: CameraIntrinsics,
    depth: CameraIntrinsics,
    absolute_tolerance: float = 1.0e-3,
) -> bool:
    """Return whether two images share dimensions and the same pinhole model."""
    rgb.validate()
    depth.validate()
    return bool(
        rgb.width == depth.width
        and rgb.height == depth.height
        and np.allclose(
            [rgb.fx, rgb.fy, rgb.cx, rgb.cy],
            [depth.fx, depth.fy, depth.cx, depth.cy],
            rtol=0.0,
            atol=absolute_tolerance,
        )
    )


def depth_to_metres(depth_image: np.ndarray, encoding: str) -> np.ndarray:
    """Convert supported ROS depth encodings to float32 metres."""
    if depth_image.ndim != 2:
        raise ValueError("Depth image must be two-dimensional")
    normalized_encoding = encoding.upper()
    if normalized_encoding == "32FC1":
        return depth_image.astype(np.float32, copy=False)
    if normalized_encoding == "16UC1":
        return depth_image.astype(np.float32) * 0.001
    raise ValueError("Unsupported depth encoding: {}".format(encoding))


def robust_depth_from_detection(
    depth_metres: np.ndarray,
    red_mask: np.ndarray,
    bbox: Sequence[int],
    min_depth: float,
    max_depth: float,
    min_valid_pixels: int,
    mad_scale: float = 3.5,
) -> Optional[DepthEstimate]:
    """Estimate depth from valid pixels in the intersection of mask and bbox.

    The estimator first applies finite/range checks, then a median absolute
    deviation filter. The returned pixel is the median location of retained
    samples rather than a single potentially invalid centre pixel.
    """
    if depth_metres.ndim != 2 or red_mask.ndim != 2:
        raise ValueError("Depth and mask must be two-dimensional")
    if depth_metres.shape != red_mask.shape:
        raise ValueError("Depth and mask dimensions differ")
    if min_depth < 0.0 or max_depth <= min_depth:
        raise ValueError("Invalid depth range")
    if min_valid_pixels < 1:
        raise ValueError("min_valid_pixels must be positive")
    if len(bbox) != 4:
        raise ValueError("bbox must contain x, y, width and height")

    x, y, width, height = [int(value) for value in bbox]
    if width <= 0 or height <= 0:
        return None
    image_height, image_width = depth_metres.shape
    x0 = max(0, x)
    y0 = max(0, y)
    x1 = min(image_width, x + width)
    y1 = min(image_height, y + height)
    if x0 >= x1 or y0 >= y1:
        return None

    depth_roi = depth_metres[y0:y1, x0:x1]
    mask_roi = red_mask[y0:y1, x0:x1] > 0
    with np.errstate(invalid="ignore"):
        valid = (
            mask_roi
            & np.isfinite(depth_roi)
            & (depth_roi > 0.0)
            & (depth_roi >= min_depth)
            & (depth_roi <= max_depth)
        )
    raw_count = int(np.count_nonzero(valid))
    if raw_count < min_valid_pixels:
        return None

    values = depth_roi[valid].astype(np.float64)
    median = float(np.median(values))
    deviations = np.abs(values - median)
    mad = float(np.median(deviations))
    if mad > np.finfo(np.float64).eps and mad_scale > 0.0:
        robust_sigma = 1.4826 * mad
        keep = deviations <= mad_scale * robust_sigma
    else:
        keep = np.ones(values.shape, dtype=bool)

    rows, columns = np.nonzero(valid)
    values = values[keep]
    rows = rows[keep]
    columns = columns[keep]
    valid_count = int(values.size)
    if valid_count < min_valid_pixels:
        return None

    return DepthEstimate(
        depth=float(np.median(values)),
        pixel_u=float(np.median(columns) + x0),
        pixel_v=float(np.median(rows) + y0),
        valid_count=valid_count,
        raw_valid_count=raw_count,
    )


def masked_depth_points(
    depth_metres: np.ndarray,
    red_mask: np.ndarray,
    bbox: Sequence[int],
    intrinsics: CameraIntrinsics,
    min_depth: float,
    max_depth: float,
    max_points: int = 4000,
) -> MaskedPointSet:
    """Extract valid masked RGB-D points without flattening sphere curvature."""
    intrinsics.validate()
    if depth_metres.shape != red_mask.shape:
        raise ValueError("Depth and mask dimensions differ")
    if depth_metres.ndim != 2 or len(bbox) != 4:
        raise ValueError("Depth/mask must be 2-D and bbox must contain four values")
    if max_points < 1 or min_depth < 0.0 or max_depth <= min_depth:
        raise ValueError("Invalid point limit or depth range")
    x, y, width, height = [int(value) for value in bbox]
    image_height, image_width = depth_metres.shape
    x0, y0 = max(0, x), max(0, y)
    x1, y1 = min(image_width, x + width), min(image_height, y + height)
    if width <= 0 or height <= 0 or x0 >= x1 or y0 >= y1:
        return MaskedPointSet(
            np.empty((0, 3), dtype=np.float64),
            np.empty((0, 2), dtype=np.float64),
            np.empty((0,), dtype=np.float64),
            0,
        )

    depth_roi = depth_metres[y0:y1, x0:x1]
    mask_roi = red_mask[y0:y1, x0:x1] > 0
    with np.errstate(invalid="ignore"):
        valid = (
            mask_roi & np.isfinite(depth_roi) & (depth_roi > 0.0)
            & (depth_roi >= min_depth) & (depth_roi <= max_depth)
        )
    rows, columns = np.nonzero(valid)
    depths = depth_roi[valid].astype(np.float64)
    raw_count = int(depths.size)
    if raw_count > max_points:
        # Deterministic uniform sampling preserves the radial depth profile.
        indices = np.linspace(0, raw_count - 1, max_points, dtype=np.int64)
        rows, columns, depths = rows[indices], columns[indices], depths[indices]
    u = columns.astype(np.float64) + x0
    v = rows.astype(np.float64) + y0
    points = np.column_stack(
        (
            (u - intrinsics.cx) * depths / intrinsics.fx,
            (v - intrinsics.cy) * depths / intrinsics.fy,
            depths,
        )
    )
    return MaskedPointSet(points, np.column_stack((u, v)), depths, raw_count)


def _initial_sphere(points: np.ndarray, mode: str, known_radius: float) -> Tuple[np.ndarray, float]:
    centroid = np.mean(points, axis=0)
    if mode == "known_radius":
        ray_norm = float(np.linalg.norm(centroid))
        ray = centroid / ray_norm if ray_norm > 1.0e-9 else np.asarray([0.0, 0.0, 1.0])
        return centroid + 0.65 * known_radius * ray, known_radius

    matrix = np.column_stack((2.0 * points, np.ones(points.shape[0])))
    target = np.einsum("ij,ij->i", points, points)
    solution, _residuals, rank, _singular = np.linalg.lstsq(matrix, target, rcond=None)
    center = solution[:3]
    radius_squared = float(np.dot(center, center) + solution[3])
    if rank < 4 or not np.isfinite(radius_squared) or radius_squared <= 0.0:
        ray_norm = float(np.linalg.norm(centroid))
        ray = centroid / ray_norm if ray_norm > 1.0e-9 else np.asarray([0.0, 0.0, 1.0])
        return centroid + 0.65 * known_radius * ray, known_radius
    return center, math.sqrt(radius_squared)


def fit_sphere_robust(
    points: np.ndarray,
    mode: str = "known_radius",
    known_radius: float = 0.15,
    min_radius: float = 0.08,
    max_radius: float = 0.25,
    max_iterations: int = 20,
    huber_delta: float = 0.008,
) -> Optional[SphereFitResult]:
    """Fit a known- or free-radius sphere with Huber IRLS Gauss-Newton."""
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape Nx3")
    points = points[np.isfinite(points).all(axis=1)]
    if points.shape[0] < 4:
        return None
    if mode not in ("known_radius", "free_radius"):
        raise ValueError("sphere fit mode must be known_radius or free_radius")
    if known_radius <= 0.0 or min_radius <= 0.0 or max_radius <= min_radius:
        raise ValueError("Invalid sphere radius constraints")
    if max_iterations < 1 or huber_delta <= 0.0:
        raise ValueError("Invalid sphere optimizer parameters")

    center, radius = _initial_sphere(points, mode, known_radius)
    if mode == "free_radius":
        radius = float(np.clip(radius, min_radius, max_radius))
    else:
        radius = float(known_radius)
    converged = False
    for _iteration in range(max_iterations):
        offsets = center[None, :] - points
        distances = np.linalg.norm(offsets, axis=1)
        usable = distances > 1.0e-9
        if np.count_nonzero(usable) < 4:
            break
        residuals = distances[usable] - radius
        absolute = np.abs(residuals)
        weights = np.ones_like(residuals)
        large = absolute > huber_delta
        weights[large] = huber_delta / np.maximum(absolute[large], 1.0e-12)
        jacobian_center = offsets[usable] / distances[usable, None]
        if mode == "free_radius":
            jacobian = np.column_stack((jacobian_center, -np.ones(residuals.size)))
        else:
            jacobian = jacobian_center
        root_weights = np.sqrt(weights)
        try:
            delta, _residuals, rank, _singular = np.linalg.lstsq(
                jacobian * root_weights[:, None],
                -residuals * root_weights,
                rcond=None,
            )
        except np.linalg.LinAlgError:
            break
        if rank < jacobian.shape[1] or not np.isfinite(delta).all():
            break
        # Bound individual updates so partial arcs cannot launch the centre away.
        center_delta = delta[:3]
        norm = float(np.linalg.norm(center_delta))
        if norm > 0.10:
            center_delta = center_delta * (0.10 / norm)
        center = center + center_delta
        if mode == "free_radius":
            radius = float(np.clip(radius + delta[3], min_radius, max_radius))
        if float(np.linalg.norm(delta)) < 1.0e-7:
            converged = True
            break

    distances = np.linalg.norm(points - center[None, :], axis=1)
    residuals = np.abs(distances - radius)
    median = float(np.median(residuals))
    mad = float(np.median(np.abs(residuals - median)))
    robust_sigma = 1.4826 * mad
    threshold = max(2.5 * robust_sigma, 2.0 * huber_delta)
    inliers = residuals <= threshold
    if np.count_nonzero(inliers) < 4:
        inliers = np.ones(points.shape[0], dtype=bool)
    return SphereFitResult(
        center=center,
        radius=radius,
        residual_rms=float(np.sqrt(np.mean(np.square(residuals[inliers])))),
        residual_median=median,
        inlier_count=int(np.count_nonzero(inliers)),
        point_count=int(points.shape[0]),
        converged=converged,
        mode=mode,
    )


def plane_fit_residual(points: np.ndarray) -> float:
    """Return orthogonal RMS residual of the best fitting plane."""
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or points.shape[0] < 3:
        return float("inf")
    centered = points - np.mean(points, axis=0)
    try:
        _u, _s, vh = np.linalg.svd(centered, full_matrices=False)
    except np.linalg.LinAlgError:
        return float("inf")
    distances = centered.dot(vh[-1])
    return float(np.sqrt(np.mean(np.square(distances))))


def radial_depth_correlation(
    pixels: np.ndarray,
    depths: np.ndarray,
    center_u: float,
    center_v: float,
) -> float:
    """Correlation of radial image distance with depth (positive for a sphere)."""
    pixels = np.asarray(pixels, dtype=np.float64)
    depths = np.asarray(depths, dtype=np.float64).reshape(-1)
    if pixels.shape != (depths.size, 2) or depths.size < 4:
        return 0.0
    radial = np.linalg.norm(pixels - np.asarray([center_u, center_v]), axis=1)
    if float(np.std(radial)) < 1.0e-9 or float(np.std(depths)) < 1.0e-9:
        return 0.0
    value = float(np.corrcoef(radial, depths)[0, 1])
    return value if np.isfinite(value) else 0.0


def validate_sphere_geometry(
    sphere: SphereFitResult,
    plane_residual_rms: float,
    radial_correlation: float,
    max_sphere_residual: float,
    max_sphere_plane_ratio: float,
    min_plane_residual: float,
    min_radial_correlation: float,
    min_inlier_ratio: float,
) -> SphereGeometryResult:
    """Validate spherical structure instead of relying on 2-D circularity."""
    inlier_ratio = float(sphere.inlier_count) / max(1, sphere.point_count)
    ratio = sphere.residual_rms / max(plane_residual_rms, 1.0e-9)
    reasons = []
    if sphere.residual_rms > max_sphere_residual:
        reasons.append("sphere_residual")
    if plane_residual_rms < min_plane_residual:
        reasons.append("too_planar")
    if ratio > max_sphere_plane_ratio:
        reasons.append("plane_preferred")
    if radial_correlation < min_radial_correlation:
        reasons.append("radial_profile")
    if inlier_ratio < min_inlier_ratio:
        reasons.append("inlier_ratio")
    residual_score = math.exp(-((sphere.residual_rms / max(max_sphere_residual, 1.0e-9)) ** 2))
    plane_score = float(np.clip(1.0 - ratio / max(max_sphere_plane_ratio, 1.0e-9), 0.0, 1.0))
    radial_score = float(np.clip(
        (radial_correlation - min_radial_correlation)
        / max(1.0 - min_radial_correlation, 1.0e-9), 0.0, 1.0
    ))
    confidence = float(np.clip(
        0.45 * residual_score + 0.25 * plane_score + 0.20 * radial_score
        + 0.10 * inlier_ratio,
        0.0,
        1.0,
    ))
    return SphereGeometryResult(
        accepted=not reasons,
        sphere=sphere,
        plane_residual_rms=float(plane_residual_rms),
        radial_depth_correlation=float(radial_correlation),
        confidence=confidence,
        reason="ok" if not reasons else ",".join(reasons),
    )


def monocular_sphere_position(
    center_u: float,
    center_v: float,
    radius_pixels: float,
    sphere_radius: float,
    intrinsics: CameraIntrinsics,
) -> Optional[np.ndarray]:
    """Optional low-confidence centre estimate from configured physical radius."""
    intrinsics.validate()
    if radius_pixels <= 0.0 or sphere_radius <= 0.0:
        return None
    focal = math.sqrt(intrinsics.fx * intrinsics.fy)
    # Perspective projection of a sphere: image radius = f*r/sqrt(z^2-r^2).
    center_depth = sphere_radius * math.sqrt(1.0 + (focal / radius_pixels) ** 2)
    return backproject_pixel(center_u, center_v, center_depth, intrinsics)


def backproject_pixel(
    pixel_u: float,
    pixel_v: float,
    depth: float,
    intrinsics: CameraIntrinsics,
) -> np.ndarray:
    """Backproject a rectified image pixel to an optical-frame XYZ point."""
    intrinsics.validate()
    if not np.isfinite((pixel_u, pixel_v, depth)).all() or depth <= 0.0:
        raise ValueError("Pixel and depth must be finite and depth must be positive")
    return np.asarray(
        [
            (float(pixel_u) - intrinsics.cx) * float(depth) / intrinsics.fx,
            (float(pixel_v) - intrinsics.cy) * float(depth) / intrinsics.fy,
            float(depth),
        ],
        dtype=np.float64,
    )


def project_points(points: np.ndarray, intrinsics: CameraIntrinsics) -> Tuple[np.ndarray, np.ndarray]:
    """Project Nx3 optical-frame points, returning pixels and valid-Z mask."""
    intrinsics.validate()
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape Nx3")
    valid = np.isfinite(points).all(axis=1) & (points[:, 2] > 0.0)
    pixels = np.full((points.shape[0], 2), np.nan, dtype=np.float64)
    z = points[valid, 2]
    pixels[valid, 0] = intrinsics.fx * points[valid, 0] / z + intrinsics.cx
    pixels[valid, 1] = intrinsics.fy * points[valid, 1] / z + intrinsics.cy
    return pixels, valid


def rasterize_point_cloud_depth(
    points: np.ndarray,
    intrinsics: CameraIntrinsics,
    min_depth: float,
    max_depth: float,
) -> np.ndarray:
    """Create a registered depth map using a nearest-point z-buffer."""
    pixels, valid = project_points(points, intrinsics)
    points = np.asarray(points, dtype=np.float64)
    valid &= (points[:, 2] >= min_depth) & (points[:, 2] <= max_depth)
    u = np.zeros(points.shape[0], dtype=np.int64)
    v = np.zeros(points.shape[0], dtype=np.int64)
    u[valid] = np.rint(pixels[valid, 0]).astype(np.int64)
    v[valid] = np.rint(pixels[valid, 1]).astype(np.int64)
    valid &= (u >= 0) & (u < intrinsics.width) & (v >= 0) & (v < intrinsics.height)
    depth = np.full((intrinsics.height, intrinsics.width), np.nan, dtype=np.float32)
    for index in np.flatnonzero(valid):
        old = depth[v[index], u[index]]
        value = float(points[index, 2])
        if not np.isfinite(old) or value < old:
            depth[v[index], u[index]] = value
    return depth
