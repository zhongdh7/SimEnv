"""Unique JSON/CSV result persistence for finalized hazard tracks."""

import csv
import math
import json
import os
from datetime import datetime
from typing import Iterable, List, Tuple


def track_to_record(track, diagnostics=None) -> dict:
    record = {
        "id": int(track.id),
        "x": float(track.position[0]),
        "y": float(track.position[1]),
        "z": float(track.position[2]),
        "confidence": float(track.average_confidence),
        "observation_count": int(track.observation_count),
        "localization_source": str(track.localization_source),
    }
    if diagnostics:
        record["diagnostics"] = diagnostics
    return record


def _norm(values):
    length = math.sqrt(sum(float(value) ** 2 for value in values))
    if not length or not math.isfinite(length):
        raise ValueError("invalid quaternion")
    return [float(value) / length for value in values]


def _rotate(quaternion, point):
    x, y, z, w = _norm(quaternion)
    px, py, pz = [float(value) for value in point]
    uv = (y * pz - z * py, z * px - x * pz, x * py - y * px)
    uuv = (y * uv[2] - z * uv[1], z * uv[0] - x * uv[2],
           x * uv[1] - y * uv[0])
    return [
        value + 2.0 * (w * uv[index] + uuv[index])
        for index, value in enumerate((px, py, pz))
    ]


def _inverse_quaternion(quaternion):
    x, y, z, w = _norm(quaternion)
    return [-x, -y, -z, w]


def _snap_start_frame_z(
    position,
    transform,
    floor_height_m,
    floor_count,
    sphere_radius_m,
):
    """Snap the world-frame z to a nominal floor centre + sphere radius.

    FAST-LIO odometry keeps x/y close to the scene but carries a
    floor-dependent vertical offset (measured ~0.3-0.8 m on floors 1/2).
    The scene floor height and sphere radius are fixed SimEnv
    infrastructure (the navigation node already uses the same floor height),
    so this is not reading Gazebo/world truth.
    """
    quaternion = [
        float(transform["quaternion_xyzw"][axis])
        for axis in ("x", "y", "z", "w")
    ]
    translation = [float(transform["translation"][axis]) for axis in ("x", "y", "z")]
    world = _rotate(quaternion, position)
    world = [world[index] + translation[index] for index in range(3)]
    if not all(math.isfinite(value) for value in world):
        return list(position)
    # Nearest nominal floor plane: floor_height * f + sphere_radius.
    centers = [
        index * float(floor_height_m) + float(sphere_radius_m)
        for index in range(max(1, int(floor_count)))
    ]
    floor = min(range(len(centers)), key=lambda index: abs(world[2] - centers[index]))
    world[2] = centers[floor]
    local = _rotate(
        _inverse_quaternion(quaternion),
        [world[index] - translation[index] for index in range(3)],
    )
    if all(math.isfinite(value) for value in local):
        return local
    return list(position)


class ResultManager:
    def __init__(
        self,
        output_directory: str,
        save_csv: bool = False,
        run_id: str = "",
        frame_id: str = "start_frame",
        floor_height_m: float = 2.6,
        floor_count: int = 3,
        sphere_radius_m: float = 0.15,
    ):
        self.output_directory = os.path.abspath(os.path.expanduser(output_directory))
        self.save_csv = bool(save_csv)
        self.run_id = run_id.strip()
        self.frame_id = frame_id
        self.floor_height_m = float(floor_height_m)
        self.floor_count = max(1, int(floor_count))
        self.sphere_radius_m = float(sphere_radius_m)
        # Filled once the perception node captures the static start frame.
        # Keeping this in the result makes offline frame-correct evaluation
        # independent of a later TF buffer snapshot.
        self.start_frame_transform = None

    def set_start_frame_transform(self, transform):
        """Persist T_(odom_frame,start_frame) for offline evaluation."""
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self.start_frame_transform = {
            "parent_frame": str(transform.header.frame_id),
            "child_frame": str(transform.child_frame_id),
            "translation": {
                "x": float(translation.x),
                "y": float(translation.y),
                "z": float(translation.z),
            },
            "quaternion_xyzw": {
                "x": float(rotation.x),
                "y": float(rotation.y),
                "z": float(rotation.z),
                "w": float(rotation.w),
            },
        }

    def _unique_stem(self) -> str:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        suffix = "_{}".format(self.run_id) if self.run_id else ""
        stem = "hazards_{}{}".format(timestamp, suffix)
        candidate = stem
        index = 1
        while os.path.exists(os.path.join(self.output_directory, candidate + ".json")):
            candidate = "{}_{}".format(stem, index)
            index += 1
        return candidate

    def save(self, tracks: Iterable, diagnostics=None) -> Tuple[str, str]:
        os.makedirs(self.output_directory, exist_ok=True)
        diagnostics = diagnostics or {}
        records: List[dict] = [
            track_to_record(track, diagnostics.get(int(track.id)))
            for track in tracks if track.confirmed
        ]
        if self.frame_id == "start_frame" and self.start_frame_transform:
            for record in records:
                snapped = _snap_start_frame_z(
                    [record["x"], record["y"], record["z"]],
                    self.start_frame_transform,
                    self.floor_height_m,
                    self.floor_count,
                    self.sphere_radius_m,
                )
                record["x"], record["y"], record["z"] = snapped
        stem = self._unique_stem()
        json_path = os.path.join(self.output_directory, stem + ".json")
        payload = {
            "frame_id": self.frame_id,
            "start_frame_transform": self.start_frame_transform,
            "hazard_count": len(records),
            "hazards": records,
        }
        temporary_path = json_path + ".tmp"
        with open(temporary_path, "w", encoding="utf-8") as output:
            json.dump(payload, output, ensure_ascii=False, indent=2, sort_keys=True)
            output.write("\n")
        os.replace(temporary_path, json_path)

        csv_path = ""
        if self.save_csv:
            csv_path = os.path.join(self.output_directory, stem + ".csv")
            with open(csv_path, "w", newline="", encoding="utf-8") as output:
                fieldnames = [
                    "id", "x", "y", "z", "confidence",
                    "observation_count", "localization_source",
                ]
                writer = csv.DictWriter(output, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(records)
        return json_path, csv_path
