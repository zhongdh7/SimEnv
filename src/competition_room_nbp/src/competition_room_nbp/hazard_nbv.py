"""Task-aware next-best-view ranking for room-local hazard search."""

from __future__ import print_function

import math


class HazardNBVSelector(object):
    """Choose a safe viewpoint by residual hazard visibility per travel cost.

    Candidates must already be known-free, in-room and A*-reachable.  This
    module intentionally has no ROS, detector, or simulator dependency.
    """

    YAW_COUNT = 8

    def __init__(self, min_gain_cells=4, max_path_m=6.0,
                 turn_cost_m=0.35, gain_bias_m=1.0):
        self.min_gain_cells = max(1.0, float(min_gain_cells))
        self.max_path_m = max(0.5, float(max_path_m))
        self.turn_cost_m = max(0.0, float(turn_cost_m))
        self.gain_bias_m = max(0.1, float(gain_bias_m))

    @staticmethod
    def _angle_distance(first, second):
        return abs(math.atan2(math.sin(first - second),
                              math.cos(first - second)))

    def select(self, pose, candidates):
        """Return the best candidate/yaw record, or ``None`` when exhausted."""
        best = None
        current_yaw = float(pose[3])
        for candidate in candidates:
            path_m = max(0.0, float(candidate.get("path_m", 0.0)))
            if path_m > self.max_path_m:
                continue
            visibility = candidate.get("hazard_visibility", ())
            if not isinstance(visibility, (list, tuple)):
                visibility = [visibility] * self.YAW_COUNT
            camera_visibility = candidate.get("camera_visibility", ())
            if not isinstance(camera_visibility, (list, tuple)):
                camera_visibility = [camera_visibility] * self.YAW_COUNT
            for yaw_channel in range(min(self.YAW_COUNT, len(visibility))):
                gain = float(visibility[yaw_channel])
                if not math.isfinite(gain) or gain < self.min_gain_cells:
                    continue
                yaw = yaw_channel * (2.0 * math.pi / self.YAW_COUNT)
                turn = self._angle_distance(yaw, current_yaw)
                effective_cost = (self.gain_bias_m + path_m +
                                  self.turn_cost_m * turn / math.pi)
                efficiency = gain / effective_cost
                score = efficiency + 0.05 * gain
                record = dict(candidate)
                record.update({
                    "strategy": "hazard_nbv",
                    "yaw_channel": int(yaw_channel),
                    "yaw": float(yaw),
                    "hazard_new_visibility": gain,
                    "camera_new_visibility": float(
                        camera_visibility[yaw_channel]
                        if yaw_channel < len(camera_visibility) else 0.0
                    ),
                    "effective_cost_m": float(effective_cost),
                    "hazard_gain_per_cost": float(efficiency),
                    "score": float(score),
                    "value": 0.0,
                    "model_ms": 0.0,
                })
                key = (record["score"], gain, -path_m, -turn)
                if best is None or key > best[0]:
                    best = (key, record)
        return None if best is None else best[1]
