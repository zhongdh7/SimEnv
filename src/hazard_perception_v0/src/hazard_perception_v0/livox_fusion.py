"""Geometric Livox ROI extraction and explainable RGB-D/Livox fusion."""

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

from hazard_perception_v0.localization import CameraIntrinsics, project_points


@dataclass(frozen=True)
class LivoxEstimate:
    position: np.ndarray
    point_count: int
    points: np.ndarray


@dataclass(frozen=True)
class FusionResult:
    position: np.ndarray
    source: str
    disagreement: Optional[float]


def _largest_cluster(points: np.ndarray, tolerance: float) -> np.ndarray:
    """Return largest Euclidean cluster; point counts in image ROIs are small."""
    if points.shape[0] <= 1 or tolerance <= 0.0:
        return points
    distances = np.linalg.norm(points[:, None, :] - points[None, :, :], axis=2)
    adjacency = distances <= tolerance
    unseen = set(range(points.shape[0]))
    clusters = []
    while unseen:
        seed = unseen.pop()
        component = {seed}
        frontier = [seed]
        while frontier:
            neighbours = set(np.flatnonzero(adjacency[frontier.pop()]).tolist()) & unseen
            unseen -= neighbours
            component |= neighbours
            frontier.extend(neighbours)
        clusters.append(sorted(component))
    return points[max(clusters, key=len)]


def estimate_livox_position(
    camera_points: np.ndarray,
    red_mask: np.ndarray,
    bbox: Sequence[int],
    intrinsics: CameraIntrinsics,
    min_range: float,
    max_range: float,
    min_points: int,
    cluster_tolerance: float,
    mad_scale: float = 3.5,
) -> Optional[LivoxEstimate]:
    points = np.asarray(camera_points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("camera_points must have shape Nx3")
    if red_mask.shape != (intrinsics.height, intrinsics.width):
        raise ValueError("Mask dimensions do not match camera model")
    if len(bbox) != 4 or min_points < 1:
        raise ValueError("Invalid bbox or minimum point count")

    pixels, valid = project_points(points, intrinsics)
    ranges = np.linalg.norm(points, axis=1)
    valid &= np.isfinite(ranges) & (ranges >= min_range) & (ranges <= max_range)
    u = np.zeros(points.shape[0], dtype=np.int64)
    v = np.zeros(points.shape[0], dtype=np.int64)
    u[valid] = np.rint(pixels[valid, 0]).astype(np.int64)
    v[valid] = np.rint(pixels[valid, 1]).astype(np.int64)
    x, y, width, height = [int(value) for value in bbox]
    valid &= (
        (u >= max(0, x))
        & (u < min(intrinsics.width, x + width))
        & (v >= max(0, y))
        & (v < min(intrinsics.height, y + height))
        & (u >= 0)
        & (u < intrinsics.width)
        & (v >= 0)
        & (v < intrinsics.height)
    )
    valid_indices = np.flatnonzero(valid)
    if valid_indices.size:
        mask_keep = red_mask[v[valid_indices], u[valid_indices]] > 0
        valid_indices = valid_indices[mask_keep]
    selected = points[valid_indices]
    if selected.shape[0] < min_points:
        return None

    selected = _largest_cluster(selected, cluster_tolerance)
    if selected.shape[0] < min_points:
        return None
    centre = np.median(selected, axis=0)
    residuals = np.linalg.norm(selected - centre, axis=1)
    median_residual = float(np.median(residuals))
    mad = float(np.median(np.abs(residuals - median_residual)))
    if mad > np.finfo(np.float64).eps and mad_scale > 0.0:
        keep = residuals <= median_residual + mad_scale * 1.4826 * mad
        selected = selected[keep]
    if selected.shape[0] < min_points:
        return None
    return LivoxEstimate(
        position=np.median(selected, axis=0),
        point_count=selected.shape[0],
        points=selected.copy(),
    )


def choose_fused_position(
    rgbd_position: Optional[np.ndarray],
    livox_position: Optional[np.ndarray],
    max_disagreement: float,
    rgbd_weight: float,
    livox_weight: float,
) -> Optional[FusionResult]:
    """Fuse compatible estimates and otherwise prefer the colour-gated RGB-D one."""
    if rgbd_position is None and livox_position is None:
        return None
    if rgbd_position is None:
        return FusionResult(np.asarray(livox_position, dtype=np.float64), "livox", None)
    if livox_position is None:
        return FusionResult(np.asarray(rgbd_position, dtype=np.float64), "rgbd", None)
    rgbd = np.asarray(rgbd_position, dtype=np.float64)
    livox = np.asarray(livox_position, dtype=np.float64)
    disagreement = float(np.linalg.norm(rgbd - livox))
    if disagreement > max_disagreement:
        return FusionResult(rgbd, "rgbd", disagreement)
    total_weight = rgbd_weight + livox_weight
    if rgbd_weight < 0.0 or livox_weight < 0.0 or total_weight <= 0.0:
        raise ValueError("Fusion weights must be non-negative with positive sum")
    fused = (rgbd_weight * rgbd + livox_weight * livox) / total_weight
    return FusionResult(fused, "fused", disagreement)
