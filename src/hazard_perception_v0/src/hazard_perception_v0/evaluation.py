"""Pure helpers for post-run hazard evaluation.

This module is offline-only.  Runtime perception, tracking, pose bridges and
finalization must never import simulator truth or evaluation annotations.  In
particular, only evaluator entry points may open ``danger_truth.json``.
"""

from functools import lru_cache
import math
import statistics


def normalize_quaternion_xyzw(quaternion):
    values = tuple(float(value) for value in quaternion)
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion must contain four finite values")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-12:
        raise ValueError("quaternion norm is zero")
    return tuple(value / norm for value in values)


def rotate_point_xyzw(quaternion, point):
    """Rotate a finite XYZ point with a normalized xyzw quaternion."""
    x, y, z, w = normalize_quaternion_xyzw(quaternion)
    px, py, pz = (float(value) for value in point)
    if not all(math.isfinite(value) for value in (px, py, pz)):
        raise ValueError("point must contain three finite values")
    uv = (y * pz - z * py, z * px - x * pz, x * py - y * px)
    uuv = (y * uv[2] - z * uv[1], z * uv[0] - x * uv[2], x * uv[1] - y * uv[0])
    return [
        value + 2.0 * (w * uv[index] + uuv[index])
        for index, value in enumerate((px, py, pz))
    ]


def transform_point(transform, point):
    """Apply persisted T_(parent,child) to a point in the child frame."""
    translation = transform["translation"]
    quaternion = transform["quaternion_xyzw"]
    rotated = rotate_point_xyzw(
        (quaternion["x"], quaternion["y"], quaternion["z"], quaternion["w"]),
        point,
    )
    result = [
        rotated[0] + float(translation["x"]),
        rotated[1] + float(translation["y"]),
        rotated[2] + float(translation["z"]),
    ]
    if not all(math.isfinite(value) for value in result):
        raise ValueError("transformed point contains NaN/Inf")
    return result


def distance(left, right):
    return math.sqrt(sum(
        (float(left[index]) - float(right[index])) ** 2 for index in range(3)
    ))


def maximum_cardinality_min_distance_matching(predictions, truth, threshold):
    """Return threshold-valid one-to-one pairs with a lexicographic objective.

    The primary objective is maximum cardinality (maximum TP).  Among all
    maximum-cardinality matchings, the secondary objective is minimum total
    Euclidean distance.  Dynamic programming masks the smaller side, which is
    practical for the small hazard sets used by SimEnv and needs no scipy.
    """
    threshold = float(threshold)
    if not math.isfinite(threshold) or threshold < 0.0:
        raise ValueError("threshold must be a finite non-negative distance")
    if not predictions or not truth:
        return []

    # Mask the smaller partition to keep the state space at 2**min(P, G).
    swapped = len(predictions) < len(truth)
    large = truth if swapped else predictions
    small = predictions if swapped else truth
    valid_edges = []
    for large_index, large_point in enumerate(large):
        edges = []
        for small_index, small_point in enumerate(small):
            error = distance(large_point, small_point)
            if error <= threshold:
                edges.append((small_index, error))
        valid_edges.append(tuple(edges))

    @lru_cache(maxsize=None)
    def solve(large_index, used_mask):
        if large_index == len(large):
            return 0, 0.0, ()
        best = solve(large_index + 1, used_mask)
        for small_index, error in valid_edges[large_index]:
            bit = 1 << small_index
            if used_mask & bit:
                continue
            count, cost, pairs = solve(large_index + 1, used_mask | bit)
            candidate = (count + 1, cost + error, ((large_index, small_index, error),) + pairs)
            if (candidate[0] > best[0]
                    or (candidate[0] == best[0] and candidate[1] < best[1] - 1.0e-12)
                    or (candidate[0] == best[0]
                        and abs(candidate[1] - best[1]) <= 1.0e-12
                        and candidate[2] < best[2])):
                best = candidate
        return best

    _count, _cost, selected = solve(0, 0)
    matches = []
    for large_index, small_index, error in selected:
        prediction_index, truth_index = (
            (small_index, large_index) if swapped else (large_index, small_index)
        )
        matches.append({
            "prediction_index": prediction_index,
            "truth_index": truth_index,
            "error_m": error,
        })
    return sorted(matches, key=lambda item: (item["prediction_index"], item["truth_index"]))


def percentile(values, percent):
    """Linear-interpolated percentile compatible with small stdlib-only runs."""
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    position = (len(ordered) - 1) * float(percent) / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def error_statistics(errors):
    values = [float(value) for value in errors]
    if not values:
        return {
            "mean_error_m": None,
            "median_error_m": None,
            "p95_error_m": None,
            "max_error_m": None,
        }
    return {
        "mean_error_m": statistics.mean(values),
        "median_error_m": statistics.median(values),
        "p95_error_m": percentile(values, 95.0),
        "max_error_m": max(values),
    }
