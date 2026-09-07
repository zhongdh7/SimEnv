#!/usr/bin/env python3
"""Deterministically replay RGB-D tracker input from a diagnostics JSONL file.

This is an offline diagnostic only.  It never supplies simulator truth to the
tracker; truth is loaded, if requested, only after replay for reporting and
one-to-one matching.  Raw camera points are passed through the same optical ->
sensor-body -> odom -> start_frame rigid chain used by production.
"""

from __future__ import print_function

import argparse
import json
import math
import os
import sys

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, "src"))

from hazard_perception_v0.localization import (  # noqa: E402
    inverse_transform_point_xyzw,
    optical_to_sensor_body,
    transform_point_xyzw,
)
from hazard_perception_v0.tracker import HazardObservation, HazardTracker  # noqa: E402


def _camera_to_odom(raw, camera_tf):
    body = optical_to_sensor_body(raw)
    return transform_point_xyzw(
        body,
        camera_tf["translation_xyz"],
        camera_tf["quaternion_xyzw"],
    )


def _load_start_transform(path):
    if not path:
        return None
    with open(path, encoding="utf-8") as source:
        payload = json.load(source)
    payload = payload.get("start_frame_transform", payload)
    t = payload["translation"]
    q = payload["quaternion_xyzw"]
    return {
        "translation": [t[axis] for axis in ("x", "y", "z")],
        "quaternion_xyzw": [q[axis] for axis in ("x", "y", "z", "w")],
    }


def replay(diagnostics_path, start_transform=None, truth_path=None,
           merge_distance=0.6, output_path=None):
    tracker = HazardTracker(
        confirmation_count=3, merge_distance=merge_distance,
        smoothing_alpha=0.35, candidate_timeout=3.0,
        max_position_jump=0.45, final_merge_distance=merge_distance,
    )
    rows = []
    events = []
    historical_groups = {}
    with open(diagnostics_path, encoding="utf-8") as source:
        for line in source:
            row = json.loads(line)
            camera_tf = row.get("T_odom_camera") or {}
            if not camera_tf:
                continue
            for candidate in row.get("candidates", []):
                if not candidate.get("geometry_validated"):
                    continue
                raw = candidate.get("raw_camera_xyz")
                if not raw or len(raw) != 3:
                    continue
                odom = _camera_to_odom(raw, camera_tf)
                if start_transform:
                    position = inverse_transform_point_xyzw(
                        odom, start_transform["translation"],
                        start_transform["quaternion_xyzw"],
                    )
                else:
                    # Identity is an explicitly named odom-as-start replay
                    # frame.  It preserves all rigid distances and tracker
                    # behavior when the historical static transform was not
                    # recorded, while avoiding the old, incorrect diagnostics
                    # start_frame_xyz field.
                    position = odom
                observation = HazardObservation(
                    position=position,
                    confidence=float(candidate.get("combined_confidence", 0.5)),
                    timestamp=float(row.get("stamp", 0.0)),
                    localization_source=str(candidate.get("source", "rgbd"))
                    if candidate.get("source", "rgbd") in ("rgbd", "livox", "fused", "monocular")
                    else "rgbd",
                    geometry_validated=True,
                    required_confirmation_count=int(
                        candidate.get("required_confirmation_count", 0) or 0
                    ),
                )
                track = tracker.update(observation)
                events.append(dict(tracker.last_event))
                rows.append({
                    "stamp": observation.timestamp,
                    "position_start": position.tolist(),
                    "position_odom": odom.tolist(),
                    "track_id": int(track.id) if track else None,
                    "event": dict(tracker.last_event),
                })
                historical_id = candidate.get("track_id")
                if historical_id is not None:
                    historical_groups.setdefault(str(historical_id), []).append(
                        position.tolist()
                    )

    before = [track for track in tracker.tracks]
    before_records = [_track_record(track) for track in before]
    before_confirmed = [track for track in before if track.confirmed]
    tracker.finalize()
    after = tracker.tracks
    after_records = [_track_record(track) for track in after]
    result = {
        "diagnostics_path": diagnostics_path,
        "position_frame": "start_frame" if start_transform else "odom_replay_start",
        "start_transform_used": start_transform,
        "input_validated_observations": len(rows),
        "tracker_events": _event_counts(events),
        "tracks_before_finalize": before_records,
        "confirmed_before_dedup": [_track_record(track) for track in before_confirmed],
        "confirmed_after_finalize_dedup": [
            _track_record(track) for track in tracker.finalize()
        ],
        "tracks_after_finalize": after_records,
        "before_finalize_count": len(before_confirmed),
        "after_finalize_count": len(tracker.finalize()),
        "track6_track10_distance": _track_distance(before, 6, 10),
        "historical_track_groups": {
            track_id: {
                "count": len(points),
                "mean_position_start": np.mean(np.asarray(points), axis=0).tolist(),
            }
            for track_id, points in sorted(historical_groups.items())
        },
    }
    if "6" in result["historical_track_groups"] and "10" in result["historical_track_groups"]:
        result["historical_track6_track10_mean_distance"] = float(np.linalg.norm(
            np.asarray(result["historical_track_groups"]["6"]["mean_position_start"])
            - np.asarray(result["historical_track_groups"]["10"]["mean_position_start"])
        ))
    if truth_path:
        result["truth_match"] = _match_after_replay(result, truth_path)
    if output_path:
        with open(output_path, "w", encoding="utf-8") as output:
            json.dump(result, output, indent=2, sort_keys=True)
            output.write("\n")
    return result


def _track_record(track):
    return {
        "id": int(track.id),
        "observation_count": int(track.observation_count),
        "validated_observation_count": int(track.validated_observation_count),
        "required_confirmation_count": int(track.required_confirmation_count),
        "confirmed": bool(track.confirmed),
        "position_start": track.position.tolist(),
        "confidence": float(track.average_confidence),
        "first_observation": float(track.first_observation),
        "last_observation": float(track.last_observation),
    }


def _event_counts(events):
    counts = {}
    for event in events:
        name = event.get("event", "none")
        counts[name] = counts.get(name, 0) + 1
    return counts


def _track_distance(tracks, first_id, second_id):
    by_id = {track.id: track for track in tracks}
    if first_id not in by_id or second_id not in by_id:
        return None
    return float(np.linalg.norm(by_id[first_id].position - by_id[second_id].position))


def _match_after_replay(result, truth_path):
    with open(truth_path, encoding="utf-8") as source:
        truth = json.load(source).get("danger_sources", [])
    gt = [np.asarray(item["position"], dtype=np.float64)
          for item in truth if int(item.get("floor_index", 0)) == 0]
    predictions = [np.asarray(item["position_start"], dtype=np.float64)
                   for item in result["confirmed_after_finalize_dedup"]]
    # This report is deliberately only a convenience for the odom replay
    # (truth is never fed back into the tracker).  Exact frame matching needs
    # a supplied start transform and is handled by evaluate_floor0.py.
    return {"gt_count": len(gt), "prediction_count": len(predictions),
            "note": "post-replay count only; use evaluate_floor0.py for frame-correct matching"}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("diagnostics")
    parser.add_argument("--start-transform")
    parser.add_argument("--truth")
    parser.add_argument("--output")
    parser.add_argument("--merge-distance", type=float, default=0.6)
    args = parser.parse_args()
    result = replay(
        args.diagnostics, _load_start_transform(args.start_transform),
        args.truth, args.merge_distance, args.output,
    )
    print(json.dumps({
        key: result[key] for key in (
            "position_frame", "input_validated_observations",
            "before_finalize_count", "after_finalize_count",
            "track6_track10_distance", "tracker_events",
            "confirmed_before_dedup", "confirmed_after_finalize_dedup",
        )
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
