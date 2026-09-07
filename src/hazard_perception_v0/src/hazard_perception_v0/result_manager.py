"""Unique JSON/CSV result persistence for finalized hazard tracks."""

import csv
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


class ResultManager:
    def __init__(
        self,
        output_directory: str,
        save_csv: bool = False,
        run_id: str = "",
        frame_id: str = "start_frame",
    ):
        self.output_directory = os.path.abspath(os.path.expanduser(output_directory))
        self.save_csv = bool(save_csv)
        self.run_id = run_id.strip()
        self.frame_id = frame_id
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
