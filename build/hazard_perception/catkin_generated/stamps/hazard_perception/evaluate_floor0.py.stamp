#!/usr/bin/env python3
"""Offline, frame-correct floor-0 hazard evaluation.

This tool is intentionally not imported by the runtime pipeline.  It may read
the simulator truth only after a run has stopped.  Prediction coordinates are
stored in ``start_frame``; the perception result now also stores the static
T_(odom,start_frame) transform captured at startup, so evaluation applies the
full translation and rotation before matching against world/odom truth.
"""

from __future__ import print_function

import argparse
import itertools
import json
import math
import os


def _q_rotate(q, p):
    x, y, z, w = [float(v) for v in q]
    # Rotation matrix for xyzw quaternion.
    r00 = 1 - 2 * (y * y + z * z)
    r01 = 2 * (x * y - z * w)
    r02 = 2 * (x * z + y * w)
    r10 = 2 * (x * y + z * w)
    r11 = 1 - 2 * (x * x + z * z)
    r12 = 2 * (y * z - x * w)
    r20 = 2 * (x * z - y * w)
    r21 = 2 * (y * z + x * w)
    r22 = 1 - 2 * (x * x + y * y)
    return [
        r00 * p[0] + r01 * p[1] + r02 * p[2],
        r10 * p[0] + r11 * p[1] + r12 * p[2],
        r20 * p[0] + r21 * p[1] + r22 * p[2],
    ]


def _transform_prediction(point, transform):
    if not transform:
        return list(point)
    t = transform["translation"]
    q = transform["quaternion_xyzw"]
    rotated = _q_rotate((q["x"], q["y"], q["z"], q["w"]), point)
    return [rotated[i] + float(t["xyz"[i]]) for i in range(3)]


def _distance(a, b):
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def _match(predictions, truth, threshold):
    """Exact minimum-distance one-to-one matching for small room sets."""
    if not predictions or not truth:
        return []
    # Enumerating the smaller side avoids a scipy dependency in the ROS image.
    if len(predictions) <= len(truth):
        best = None
        for selected in itertools.permutations(range(len(truth)), len(predictions)):
            pairs = [(i, selected[i]) for i in range(len(predictions))]
            cost = sum(_distance(predictions[i]["xyz"], truth[j]["xyz"]) for i, j in pairs)
            if best is None or cost < best[0]:
                best = (cost, pairs)
    else:
        best = None
        for selected in itertools.permutations(range(len(predictions)), len(truth)):
            pairs = [(selected[j], j) for j in range(len(truth))]
            cost = sum(_distance(predictions[i]["xyz"], truth[j]["xyz"]) for i, j in pairs)
            if best is None or cost < best[0]:
                best = (cost, pairs)
    return [
        {"prediction_index": i, "truth_index": j,
         "error_m": _distance(predictions[i]["xyz"], truth[j]["xyz"])}
        for i, j in best[1]
        if _distance(predictions[i]["xyz"], truth[j]["xyz"]) <= threshold
    ]


def evaluate(prediction_path, truth_path, output_path=None, threshold=0.75):
    with open(prediction_path, encoding="utf-8") as source:
        prediction_payload = json.load(source)
    with open(truth_path, encoding="utf-8") as source:
        truth_payload = json.load(source)

    prediction_frame = prediction_payload.get("frame_id", "unknown")
    transform = prediction_payload.get("start_frame_transform")
    raw_predictions = prediction_payload.get("hazards", [])
    predictions = []
    for item in raw_predictions:
        raw = [float(item[axis]) for axis in ("x", "y", "z")]
        xyz = _transform_prediction(raw, transform) if prediction_frame == "start_frame" else raw
        predictions.append(dict(item, raw_xyz=raw, xyz=xyz))

    truth_sources = truth_payload.get("danger_sources", [])
    truth = []
    for item in truth_sources:
        floor_index = item.get("floor_index")
        if floor_index is None:
            floor_index = int(round((float(item["position"][2]) - 0.15) / 2.6))
        if int(floor_index) == 0:
            truth.append({"id": item.get("id"), "xyz": list(item["position"])})
    # Runtime must not know floor truth; this filter exists only in this tool.
    predictions = [item for item in predictions if item["xyz"][2] < 1.3]
    matches = _match(predictions, truth, float(threshold))
    matched_predictions = {item["prediction_index"] for item in matches}
    matched_truth = {item["truth_index"] for item in matches}
    errors = [item["error_m"] for item in matches]
    report = {
        "prediction_frame": prediction_frame,
        "GT_frame": "world",
        "transform_used": transform,
        "floor_index": 0,
        "threshold_m": float(threshold),
        "gt_count": len(truth),
        "prediction_count": len(predictions),
        "TP": len(matches),
        "FP": len(predictions) - len(matched_predictions),
        "FN": len(truth) - len(matched_truth),
        "precision": (len(matches) / float(len(predictions))) if predictions else 0.0,
        "recall": (len(matches) / float(len(truth))) if truth else 1.0,
        "f1": (2.0 * len(matches) / float(len(predictions) + len(truth)))
            if predictions or truth else 1.0,
        "matches": matches,
        "mean_error_m": (sum(errors) / len(errors)) if errors else None,
        "median_error_m": (sorted(errors)[len(errors) // 2]) if errors else None,
        "p95_error_m": (sorted(errors)[min(len(errors) - 1, int(math.ceil(.95 * len(errors))) - 1)]) if errors else None,
        "max_error_m": max(errors) if errors else None,
        "predictions": predictions,
        "truth": truth,
    }
    if output_path:
        with open(output_path, "w", encoding="utf-8") as output:
            json.dump(report, output, indent=2, sort_keys=True)
            output.write("\n")
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("predictions")
    parser.add_argument("truth")
    parser.add_argument("--output", default="")
    parser.add_argument("--threshold", type=float, default=0.75)
    args = parser.parse_args()
    report = evaluate(args.predictions, args.truth, args.output or None, args.threshold)
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
