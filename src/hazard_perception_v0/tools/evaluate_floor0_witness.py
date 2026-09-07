#!/usr/bin/env python3
"""Offline witness-viewpoint audit for a floor-0 hazard.

This tool is deliberately post-run only.  It never runs in a ROS node and it
does not provide any value back to navigation.  Room bounds and the recorded
candidate pool are used to answer a narrow question: did the runtime ever
offer a safe, short, camera-facing view of the target, and which yaw(s) would
have provided one under the measured CameraInfo/extrinsic?

When an occupancy ``.npz`` is supplied it is used to reject occupied and
unknown samples.  Without one, the report labels safety as
``room_bounds_only`` rather than silently claiming traversability.
"""

from __future__ import print_function

import argparse
import json
import math
import os


def _load_jsonl(path):
    rows = []
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def _q_matrix(q):
    x, y, z, w = [float(v) for v in q]
    return (
        (1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)),
        (2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)),
        (2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)),
    )


def _world_to_camera(point, transform):
    t = transform["translation_xyz"]
    r = _q_matrix(transform["quaternion_xyzw"])
    d = [float(point[i]) - float(t[i]) for i in range(3)]
    return [sum(r[j][i] * d[j] for j in range(3)) for i in range(3)]


def _transform_from_row(row):
    value = row.get("T_odom_camera")
    if not value:
        return None
    return {
        "translation_xyz": value["translation_xyz"],
        "quaternion_xyzw": value["quaternion_xyzw"],
    }


def _truth_floor0(path):
    payload = json.load(open(path, encoding="utf-8"))
    result = []
    for source in payload.get("danger_sources", []):
        floor = source.get("floor_index")
        if floor is None:
            floor = int(round((float(source["position"][2]) - .15) / 2.6))
        if int(floor) == 0:
            result.append({
                "id": source.get("id"),
                "xyz": [float(v) for v in source["position"]],
                "radius": float(source.get("radius", .15)),
                "room_id": source.get("room_id"),
            })
    return result


def _camera_info(rows):
    for row in rows:
        info = row.get("camera_info") or {}
        if float(info.get("fx", 0.0)) > 0 and float(info.get("fy", 0.0)) > 0:
            return {
                "fx": float(info["fx"]), "fy": float(info["fy"]),
                "cx": float(info["cx"]), "cy": float(info["cy"]),
                "width": int(info.get("width", 640)),
                "height": int(info.get("height", 480)),
            }
    return {"fx": 0., "fy": 0., "cx": 320., "cy": 240.,
            "width": 640, "height": 480}


def _rooms(rows, requested=None):
    rooms = {}
    for row in rows:
        if row.get("event") != "room_start":
            continue
        room_id = int(row["room_id"])
        if requested is not None and room_id != requested:
            continue
        rooms[room_id] = dict(row.get("bounds") or {})
    return rooms


def _runtime_candidates(rows, room_id):
    result = []
    for row in rows:
        if int(row.get("room_id", -1)) != room_id:
            continue
        if row.get("event") not in ("candidate_pool", "plan"):
            continue
        values = row.get("candidates")
        if values is None and row.get("target") is not None:
            values = [{"world": row["target"][:2], "yaw": row["target"][2],
                       "path_m": row.get("path_m", None), "selected": True}]
        for item in values or []:
            if item.get("world") is None:
                continue
            result.append({
                "event": row.get("event"),
                "world": [float(item["world"][0]), float(item["world"][1])],
                "yaw": float(item.get("yaw", 0.0)),
                "path_m": item.get("path_m"),
                "selected": bool(item.get("selected", row.get("event") == "plan")),
            })
    return result


def _project_from_planar(target, viewpoint, yaw, camera_z=.30, radius=.15,
                         info=None):
    """Approximate planar base->camera projection for candidate comparison."""
    info = info or {"fx": 0., "fy": 0., "cx": 320., "cy": 240.,
                    "width": 640, "height": 480}
    dx = float(target[0]) - float(viewpoint[0])
    dy = float(target[1]) - float(viewpoint[1])
    forward = math.cos(yaw) * dx + math.sin(yaw) * dy
    lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
    vertical = float(target[2]) - float(camera_z)
    distance = math.sqrt(forward * forward + lateral * lateral + vertical * vertical)
    if forward <= 0 or info["fx"] <= 0:
        return {"distance_m": distance, "in_fov": False}
    u = info["fx"] * lateral / forward + info["cx"]
    v = info["fy"] * vertical / forward + info["cy"]
    in_fov = 0 <= u < info["width"] and 0 <= v < info["height"]
    return {"distance_m": distance, "u": u, "v": v, "in_fov": in_fov,
            "reliable_1_5m": in_fov and 1.0 <= distance <= 5.0,
            "projected_diameter_px": (info["fx"] + info["fy"]) * radius / forward}


def audit(nbp_path, truth_path, room_id=None, step=.8, min_range=1., max_range=5.,
          fov_deg=60., output_path=None, diagnostics_path=None):
    rows = _load_jsonl(nbp_path)
    info_rows = (_load_jsonl(diagnostics_path)
                 if diagnostics_path else rows)
    info = _camera_info(info_rows)
    truth = _truth_floor0(truth_path)
    rooms = _rooms(rows, room_id)
    if not rooms:
        raise ValueError("no room_start bounds found in %s" % nbp_path)
    # If truth has no room id, evaluate against every room and retain the
    # room(s) with the best witness count.  This is an audit convenience only.
    reports = []
    half_fov = math.radians(fov_deg) * .5
    for target in truth:
        room_reports = []
        selected_room_ids = ([int(target["room_id"])] if target.get("room_id") in rooms
                             else sorted(rooms))
        for rid in selected_room_ids:
            bounds = rooms[rid]
            witnesses = []
            x_min = float(bounds["x_min"]) + .35
            x_max = float(bounds["x_max"]) - .35
            y_min = float(bounds["y_min"]) + .35
            y_max = float(bounds["y_max"]) - .35
            x = x_min
            while x <= x_max + 1e-6:
                y = y_min
                while y <= y_max + 1e-6:
                    distance_xy = math.hypot(target["xyz"][0] - x,
                                             target["xyz"][1] - y)
                    if min_range <= distance_xy <= max_range:
                        bearing = math.atan2(target["xyz"][1] - y,
                                             target["xyz"][0] - x)
                        for channel in range(8):
                            yaw = channel * 2.0 * math.pi / 8.0
                            error = math.atan2(math.sin(bearing - yaw),
                                               math.cos(bearing - yaw))
                            if abs(error) <= half_fov:
                                projection = _project_from_planar(
                                    target["xyz"], (x, y), yaw, info=info,
                                    radius=target["radius"])
                                if projection.get("reliable_1_5m"):
                                    witnesses.append({
                                        "room_id": rid, "x": x, "y": y,
                                        "yaw": yaw, "reachable": "unknown",
                                        "safe": "room_bounds_only",
                                        **projection,
                                    })
                    y += float(step)
                x += float(step)
            candidates = _runtime_candidates(rows, rid)
            covered = []
            for candidate in candidates:
                projection = _project_from_planar(
                    target["xyz"], candidate["world"], candidate["yaw"], info=info,
                    radius=target["radius"])
                if projection.get("reliable_1_5m"):
                    covered.append(dict(candidate, **projection))
            selected = [item for item in covered if item.get("selected")]
            room_reports.append({
                "room_id": rid, "bounds": bounds,
                "witness_viewpoint_count": len(witnesses),
                "runtime_candidate_count": len(candidates),
                "runtime_candidate_witness_count": len(covered),
                "runtime_selected_witness_count": len(selected),
                "witness_viewpoints": witnesses,
                "runtime_candidates": candidates,
                "runtime_witness_candidates": covered,
            })
        reports.append({"id": target["id"], "gt_xyz": target["xyz"],
                        "rooms": room_reports})
    result = {
        "nbp_path": nbp_path, "truth_path": truth_path,
        "diagnostics_path": diagnostics_path,
        "camera_info": info, "fov_deg": fov_deg,
        "range_m": [min_range, max_range],
        "safety_policy": "room_bounds_only; occupancy not recorded in NBP trace",
        "hazards": reports,
    }
    if output_path:
        with open(output_path, "w", encoding="utf-8") as stream:
            json.dump(result, stream, indent=2, sort_keys=True)
            stream.write("\n")
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("nbp_log")
    parser.add_argument("truth")
    parser.add_argument("--room-id", type=int, default=None)
    parser.add_argument("--step", type=float, default=.8)
    parser.add_argument("--output", default="")
    parser.add_argument("--diagnostics", default="")
    args = parser.parse_args()
    result = audit(args.nbp_log, args.truth, room_id=args.room_id,
                   step=args.step, output_path=args.output or None,
                   diagnostics_path=args.diagnostics or None)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
