#!/usr/bin/env python3
"""Independently finalize and inspect one hazard pipeline result.

This tool calls only the public Trigger service and reads the newly finalized
sensor-result JSON.  It never reads navigation state, maps, PCDs, verifier
reports, simulator truth or Gazebo model state.
"""

import argparse
import glob
import json
import math
import os
import re
import sys
import time

import rospy
from std_srvs.srv import Trigger


def _rotate(quaternion, point):
    values = [float(value) for value in quaternion]
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("invalid persisted quaternion")
    x, y, z, w = [value / norm for value in values]
    px, py, pz = [float(value) for value in point]
    uv = (y * pz - z * py, z * px - x * pz, x * py - y * px)
    uuv = (y * uv[2] - z * uv[1], z * uv[0] - x * uv[2], x * uv[1] - y * uv[0])
    return [
        value + 2.0 * (w * uv[index] + uuv[index])
        for index, value in enumerate((px, py, pz))
    ]


def transform_hazards(payload, map_frame):
    source_frame = str(payload.get("frame_id", ""))
    hazards = [dict(item) for item in payload.get("hazards", [])]
    if source_frame == map_frame:
        return hazards
    transform = payload.get("start_frame_transform")
    if not transform:
        raise ValueError("result has no persisted start_frame_transform")
    if source_frame != str(transform.get("child_frame")):
        raise ValueError("result frame does not match transform child frame")
    if map_frame != str(transform.get("parent_frame")):
        raise ValueError("requested map frame does not match transform parent frame")
    translation = transform["translation"]
    quaternion = transform["quaternion_xyzw"]
    q = (quaternion["x"], quaternion["y"], quaternion["z"], quaternion["w"])
    t = (translation["x"], translation["y"], translation["z"])
    for hazard in hazards:
        rotated = _rotate(q, (hazard["x"], hazard["y"], hazard["z"]))
        xyz = [rotated[index] + float(t[index]) for index in range(3)]
        if not all(math.isfinite(value) for value in xyz):
            raise ValueError("transformed hazard contains NaN/Inf")
        hazard["x"], hazard["y"], hazard["z"] = xyz
        hazard["source_frame"] = source_frame
    return hazards


def _atomic_json(path, payload):
    path = os.path.abspath(os.path.expanduser(path))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    temporary = path + ".tmp"
    with open(temporary, "w", encoding="utf-8") as output:
        json.dump(payload, output, ensure_ascii=False, indent=2, sort_keys=True)
        output.write("\n")
    os.replace(temporary, path)


def _candidate_paths(directory, run_id):
    # ResultManager uses hazards_YYYYmmdd_HHMMSS_microseconds[_run].json.
    # Restrict the scan so map-frame exports such as hazards_in_odom.json can
    # never be mistaken for a newly finalized sensor result.
    result_name = re.compile(r"^hazards_\d{8}_\d{6}_\d{6}(?:_.+)?\.json$")
    paths = [
        path for path in glob.glob(os.path.join(directory, "hazards_*.json"))
        if not path.endswith(".tmp") and result_name.match(os.path.basename(path))
    ]
    if run_id:
        paths = [path for path in paths if "_{}".format(run_id) in os.path.basename(path)]
    return set(paths)


def _result_path(directory, run_id, before, service_message, timeout):
    deadline = time.time() + timeout
    while time.time() <= deadline and not rospy.is_shutdown():
        current = _candidate_paths(directory, run_id)
        created = current - before
        if created:
            return max(created, key=os.path.getmtime)
        matches = re.findall(r"[^\s]+\.json", service_message)
        for match in matches:
            candidate = match.rstrip(".,")
            if os.path.isfile(candidate):
                return os.path.abspath(candidate)
        if current and "Already finalized" in service_message:
            return max(current, key=os.path.getmtime)
        time.sleep(0.1)
    return ""


def run(args):
    directory = os.path.abspath(os.path.expanduser(args.output_directory))
    before = _candidate_paths(directory, args.run_id)
    rospy.wait_for_service(args.service, timeout=args.service_timeout)
    response = rospy.ServiceProxy(args.service, Trigger)()
    if not response.success:
        raise RuntimeError(response.message)
    path = _result_path(directory, args.run_id, before, response.message, args.result_timeout)
    if not path:
        raise RuntimeError("finalize succeeded but no hazards JSON was found")
    with open(path, encoding="utf-8") as source:
        payload = json.load(source)
    hazards = payload.get("hazards", [])
    print("Confirmed hazards: {}".format(payload.get("hazard_count", len(hazards))))
    print("Result JSON: {}".format(path))
    print("Frame: {}".format(payload.get("frame_id", "unknown")))
    for hazard in hazards:
        print("  id={} xyz=({:.3f}, {:.3f}, {:.3f})".format(
            hazard.get("id", "?"), float(hazard["x"]),
            float(hazard["y"]), float(hazard["z"])))
    if args.map_frame:
        transformed = transform_hazards(payload, args.map_frame)
        print("{} coordinates:".format(args.map_frame))
        for hazard in transformed:
            print("  id={} xyz=({:.3f}, {:.3f}, {:.3f})".format(
                hazard.get("id", "?"), float(hazard["x"]),
                float(hazard["y"]), float(hazard["z"])))
        if args.map_output:
            _atomic_json(args.map_output, {
                "schema": "hazard_map_coordinates_v1",
                "frame_id": args.map_frame,
                "source_path": path,
                "hazard_count": len(transformed),
                "hazards": transformed,
            })
            print("Map-frame JSON: {}".format(
                os.path.abspath(os.path.expanduser(args.map_output))))
    elif args.map_output:
        raise ValueError("--map-output requires --map-frame")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-directory", default="~/SimEnv/results/hazard_perception_v0")
    parser.add_argument("--run-id", default="")
    parser.add_argument("--service", default="/hazard_perception_v0/finalize")
    parser.add_argument("--service-timeout", type=float, default=15.0)
    parser.add_argument("--result-timeout", type=float, default=10.0)
    parser.add_argument("--map-frame", default="")
    parser.add_argument("--map-output", default="")
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("hazard_finalize_cli", anonymous=True, disable_signals=True)
    try:
        return run(args)
    except (OSError, KeyError, TypeError, ValueError, RuntimeError,
            rospy.ROSException, rospy.ServiceException, json.JSONDecodeError) as error:
        print("hazard finalize failed: {}".format(error), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
