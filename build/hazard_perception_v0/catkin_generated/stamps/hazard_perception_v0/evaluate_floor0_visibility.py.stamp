#!/usr/bin/env python3
"""Offline floor-0 GT visibility and perception-funnel audit.

The script is intentionally independent of the runtime pipeline.  It may be
run only after a simulation has stopped, with the recorded geometry diagnostics
and simulator truth.  The runtime contributes only real CameraInfo and TF
metadata; no truth is read by or fed back into navigation/perception.
"""

from __future__ import print_function

import argparse
import collections
import json
import math


def _q_matrix(q):
    x, y, z, w = [float(v) for v in q]
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def _world_to_camera(point, transform):
    """Apply inverse T_odom_camera to an odom/world point."""
    t = transform["translation_xyz"]
    r = _q_matrix(transform["quaternion_xyzw"])
    d = [float(point[i]) - float(t[i]) for i in range(3)]
    camera = [sum(r[j][i] * d[j] for j in range(3)) for i in range(3)]
    # SimEnv's OpenNI plugin publishes CameraInfo/image pixels with ROS
    # optical axes, but sets frameName=real_sense.  The URDF fixed joint
    # real_sense -> real_sense_optical_frame has rpy(-pi/2,0,-pi/2); apply
    # its inverse here so z is the optical forward axis used by projection.
    # Older synthetic/recorded diagnostics may omit child_frame; preserve the
    # historical camera convention in that case.  Real SimEnv rows include
    # child_frame=real_sense and therefore receive the optical-axis correction.
    child_frame = str(transform.get("child_frame", ""))
    if child_frame and not child_frame.endswith("optical_frame"):
        camera = [-camera[1], -camera[2], camera[0]]
    return camera


def _camera_to_world(point, transform):
    """Map an optical-axis camera point into the recorded odom/world frame."""
    p = [float(v) for v in point]
    child_frame = str(transform.get("child_frame", ""))
    if child_frame and not child_frame.endswith("optical_frame"):
        # Runtime geometry uses optical axes while the TF child is the body
        # frame, matching the conversion used by _point_to_start_frame().
        p = [p[2], -p[0], -p[1]]
    r = _q_matrix(transform["quaternion_xyzw"])
    t = transform["translation_xyz"]
    return [sum(r[i][j] * p[j] for j in range(3)) + float(t[i])
            for i in range(3)]


def _dist(a, b):
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def _load_jsonl(path):
    rows = []
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def _candidate_matches(candidates, u, v, width, height):
    matches = []
    for candidate in candidates:
        bbox = candidate.get("bbox") or []
        center = candidate.get("center")
        if len(bbox) >= 4:
            cx = float(bbox[0]) + 0.5 * float(bbox[2])
            cy = float(bbox[1]) + 0.5 * float(bbox[3])
            radius = max(35.0, 0.75 * math.hypot(float(bbox[2]), float(bbox[3])))
        elif center and len(center) >= 2:
            cx, cy, radius = float(center[0]), float(center[1]), 60.0
        else:
            continue
        # A red candidate can be slightly offset from the projected GT centre
        # because of sphere mask/depth alignment.  Keep this threshold explicit.
        if math.hypot(cx - u, cy - v) <= radius:
            matches.append(candidate)
    return matches


def _floor0_truth(truth_payload):
    result = []
    for source in truth_payload.get("danger_sources", []):
        floor = source.get("floor_index")
        if floor is None:
            floor = int(round((float(source["position"][2]) - 0.15) / 2.6))
        if int(floor) == 0:
            result.append({
                "id": source.get("id"),
                "xyz": [float(v) for v in source["position"]],
                "radius": float(source.get("radius", 0.15)),
                "room_id": source.get("room_id"),
            })
    return result


def _confirmed_points(prediction_payload):
    transform_payload = prediction_payload.get("start_frame_transform")
    transform = None
    if transform_payload:
        translation_payload = transform_payload["translation"]
        transform = {
            "translation_xyz": [
                translation_payload[key] for key in ("x", "y", "z")
            ] if "xyz" not in translation_payload else translation_payload["xyz"],
            "quaternion_xyzw": [
                transform_payload["quaternion_xyzw"][key]
                for key in ("x", "y", "z", "w")
            ],
        }
    points = []
    for item in prediction_payload.get("hazards", []):
        xyz = [float(item[key]) for key in ("x", "y", "z")]
        if prediction_payload.get("frame_id") == "start_frame" and transform:
            r = _q_matrix(transform["quaternion_xyzw"])
            p = [sum(r[i][j] * xyz[j] for j in range(3)) + transform["translation_xyz"][i]
                 for i in range(3)]
        else:
            p = xyz
        points.append({"id": item.get("id"), "xyz": p, "raw_xyz": xyz})
    return points


def audit(diagnostics_path, truth_path, prediction_path=None, output_path=None,
          default_fx=0.0, default_fy=0.0, default_cx=0.0, default_cy=0.0,
          default_width=640, default_height=480):
    diagnostics = _load_jsonl(diagnostics_path)
    with open(truth_path, encoding="utf-8") as stream:
        truth_payload = json.load(stream)
    truth = _floor0_truth(truth_payload)
    predictions = []
    if prediction_path:
        with open(prediction_path, encoding="utf-8") as stream:
            predictions = _confirmed_points(json.load(stream))

    reports = []
    for target in truth:
        report = {
            "id": target["id"], "room_id": target.get("room_id"),
            "gt_xyz": target["xyz"],
            "visible_frame_count": 0, "visible_1_5m_frame_count": 0,
            "candidate_frame_count": 0, "depth_valid_frame_count": 0,
            "geometry_attempt_count": 0, "geometry_pass_count": 0,
            "tracker_observation_count": 0, "confirmed": False,
            "geometry_reject_reasons": {}, "visibility_samples": [],
            "missing_pose_metadata_count": 0, "occlusion": "unknown",
        }
        tracks = collections.Counter()
        track_samples = collections.defaultdict(list)
        for row in diagnostics:
            camera_tf = row.get("T_odom_camera")
            intr = row.get("camera_info") or {}
            if not camera_tf:
                report["missing_pose_metadata_count"] += 1
                continue
            fx = float(intr.get("fx", default_fx)); fy = float(intr.get("fy", default_fy))
            cx = float(intr.get("cx", default_cx)); cy = float(intr.get("cy", default_cy))
            width = int(intr.get("width", row.get("image_width", default_width)))
            height = int(intr.get("height", row.get("image_height", default_height)))
            if fx <= 0 or fy <= 0:
                continue
            pc = _world_to_camera(target["xyz"], camera_tf)
            distance = math.sqrt(sum(v * v for v in pc))
            if pc[2] <= 0:
                continue
            u, v = fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy
            in_fov = 0 <= u < width and 0 <= v < height
            if not in_fov:
                continue
            report["visible_frame_count"] += 1
            reliable = 1.0 <= distance <= 5.0
            if reliable:
                report["visible_1_5m_frame_count"] += 1
            margin = min(u, v, width - 1 - u, height - 1 - v)
            diameter = 2.0 * 0.5 * (fx + fy) / (2.0 * pc[2]) * target["radius"]
            matches = _candidate_matches(row.get("candidates", []), u, v, width, height)
            sample = {
                "stamp": row.get("stamp"), "u": u, "v": v,
                "camera_xyz": pc, "distance_m": distance,
                "reliable_1_5m": reliable, "edge_margin_px": margin,
                "projected_diameter_px": diameter,
                "candidate_count": len(matches), "occlusion": "unknown",
            }
            if matches:
                report["candidate_frame_count"] += 1
                for candidate in matches:
                    depth_valid = int(candidate.get("depth_valid_pixels") or 0)
                    if depth_valid > 0:
                        report["depth_valid_frame_count"] += 1
                    geometry_status = str(candidate.get("geometry_status", ""))
                    attempted = geometry_status not in ("", "unavailable")
                    if attempted:
                        report["geometry_attempt_count"] += 1
                    passed = geometry_status == "passed" or bool(candidate.get("geometry_validated"))
                    if passed:
                        report["geometry_pass_count"] += 1
                    elif attempted:
                        reason = str(candidate.get("geometry_reason", "other"))
                        report["geometry_reject_reasons"][reason] = (
                            report["geometry_reject_reasons"].get(reason, 0) + 1
                        )
                    track_id = candidate.get("track_id")
                    if track_id is not None:
                        key = str(track_id)
                        tracks[key] += 1
                        track_samples[key].append({
                            "stamp": row.get("stamp"),
                            "position": candidate.get("track_position_xyz")
                            or candidate.get("start_frame_xyz"),
                            "world_position": (
                                _camera_to_world(candidate["raw_camera_xyz"], camera_tf)
                                if candidate.get("raw_camera_xyz") else None
                            ),
                            "observation_count": candidate.get("track_observation_count"),
                            "validated_observation_count": candidate.get(
                                "track_validated_observation_count"
                            ),
                            "required_confirmation_count": candidate.get(
                                "track_required_confirmation_count",
                                candidate.get("required_confirmation_count"),
                            ),
                            "confirmed": bool(candidate.get(
                                "track_confirmed", candidate.get("confirmed", False)
                            )),
                            "touches_border": bool(candidate.get("touches_border", False)),
                            "tracker_event": candidate.get("tracker_event") or {},
                        })
            report["visibility_samples"].append(sample)
        report["tracker_observation_count"] = sum(tracks.values())
        report["tracker_ids"] = dict(tracks)
        per_track = {}
        for track_id, samples in track_samples.items():
            stamps = [float(item["stamp"]) for item in samples
                      if item.get("stamp") is not None]
            positions = [item["position"] for item in samples
                         if item.get("position") and len(item["position"]) >= 3]
            world_positions = [item["world_position"] for item in samples
                               if item.get("world_position") and
                               len(item["world_position"]) >= 3]
            jumps = []
            world_jumps = []
            gaps = []
            for previous, current in zip(samples, samples[1:]):
                if (previous.get("position") and current.get("position") and
                        len(previous["position"]) >= 3 and len(current["position"]) >= 3):
                    jumps.append(_dist(previous["position"], current["position"]))
                if (previous.get("world_position") and current.get("world_position") and
                        len(previous["world_position"]) >= 3 and
                        len(current["world_position"]) >= 3):
                    world_jumps.append(_dist(previous["world_position"],
                                             current["world_position"]))
                if previous.get("stamp") is not None and current.get("stamp") is not None:
                    gaps.append(float(current["stamp"]) - float(previous["stamp"]))
            required = max(int(item.get("required_confirmation_count") or 0)
                           for item in samples)
            observation_count = max(
                int(item.get("observation_count") or 0) for item in samples
            ) or len(samples)
            validated_count = max(
                int(item.get("validated_observation_count") or 0) for item in samples
            ) or len(samples)
            if positions:
                mean = [sum(float(p[i]) for p in positions) / len(positions)
                        for i in range(3)]
                std = [math.sqrt(sum((float(p[i]) - mean[i]) ** 2
                                      for p in positions) / len(positions))
                       for i in range(3)]
            else:
                mean, std = [], []
            world_errors = [_dist(item, target["xyz"])
                            for item in world_positions]
            events = [item.get("tracker_event", {}).get("event")
                      for item in samples if item.get("tracker_event")]
            reasons = [event for event in events
                       if event in ("new_track", "rejected_jump", "expired",
                                    "finalized_reject")]
            per_track[track_id] = {
                "track_id": int(track_id) if str(track_id).isdigit() else track_id,
                "matched_frame_count": len(samples),
                "first_time": min(stamps) if stamps else None,
                "last_time": max(stamps) if stamps else None,
                "duration": (max(stamps) - min(stamps)) if stamps else 0.0,
                "required_confirmation_count": required,
                "observation_count": observation_count,
                "validated_observation_count": validated_count,
                "touches_border_count": sum(1 for item in samples
                                             if item["touches_border"]),
                "normal_frame_count": sum(1 for item in samples
                                           if not item["touches_border"]),
                "position_mean_xyz": mean,
                "position_std_xyz": std,
                "position_frame": "start_frame",
                "world_position_mean_xyz": (
                    [sum(float(p[i]) for p in world_positions) / len(world_positions)
                     for i in range(3)] if world_positions else []
                ),
                "world_position_error_mean_m": (
                    sum(world_errors) / len(world_errors) if world_errors else None
                ),
                "world_position_error_max_m": (
                    max(world_errors) if world_errors else None
                ),
                "max_consecutive_position_jump": max(jumps) if jumps else 0.0,
                "max_consecutive_world_position_jump": (
                    max(world_jumps) if world_jumps else 0.0
                ),
                "max_timestamp_gap": max(gaps) if gaps else 0.0,
                "confirmed": any(bool(item["confirmed"]) for item in samples),
                "events": events,
                "expiration_new_track_reason": reasons,
            }
        # Expiration is emitted on the first observation that follows the
        # timeout, so attach that event to the expired track rather than to
        # only the newly-created replacement track.
        for samples in track_samples.values():
            for item in samples:
                event = item.get("tracker_event") or {}
                for expired_id in event.get("expired_track_ids", []):
                    key = str(expired_id)
                    if key in per_track:
                        reasons = per_track[key]["expiration_new_track_reason"]
                        if "expired" not in reasons:
                            reasons.append("expired")
        report["track_count"] = len(per_track)
        report["max_observations_single_track"] = max(
            (item["observation_count"] for item in per_track.values()), default=0
        )
        report["tracks"] = per_track
        report["tracker_confirmed"] = any(
            bool(item.get("confirmed")) for item in per_track.values()
        )
        report["tracker_confirmed_gt_match"] = any(
            bool(item.get("confirmed")) and
            item.get("world_position_error_mean_m") is not None and
            float(item["world_position_error_mean_m"]) <= 0.75
            for item in per_track.values()
        )
        if predictions:
            report["confirmed"] = min(_dist(item["xyz"], target["xyz"]) for item in predictions) <= 0.75
        if report["visible_frame_count"] == 0:
            classification = "never_visible"
        elif report["candidate_frame_count"] == 0:
            classification = "visible_no_2d_candidate"
        elif report["depth_valid_frame_count"] == 0:
            classification = "candidate_but_no_depth"
        elif report["geometry_pass_count"] == 0:
            classification = "depth_but_geometry_rejected"
        elif not report["confirmed"]:
            classification = "geometry_passed_but_not_confirmed"
        else:
            classification = "confirmed"
        report["classification"] = classification
        reports.append(report)

    result = {
        "diagnostics_path": diagnostics_path, "truth_path": truth_path,
        "prediction_path": prediction_path, "GT_frame": "world",
        "camera_pose_frame": "odom (recorded T_odom_camera)",
        "transform_used": (
            "inverse(T_odom_camera), then real_sense->optical axes "
            "(x_opt,y_opt,z_opt)=(-y_real,-z_real,x_real)"
        ),
        "occlusion_policy": "unknown (no raw depth map is persisted)",
        "gt_count": len(reports), "hazards": reports,
    }
    if output_path:
        with open(output_path, "w", encoding="utf-8") as stream:
            json.dump(result, stream, indent=2, sort_keys=True)
            stream.write("\n")
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("diagnostics")
    parser.add_argument("truth")
    parser.add_argument("--predictions", default="")
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    result = audit(args.diagnostics, args.truth, args.predictions or None, args.output or None)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
