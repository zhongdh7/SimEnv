#!/usr/bin/env python3
"""Offline all-floor hazard score.

This evaluator is intentionally outside the runtime graph. It may read
danger_truth.json only after navigation has stopped and scores the persisted
sensor result against every danger source. Nothing from this file is imported
by navigation or hazard perception.
"""

from __future__ import print_function

import argparse
import json
import math

from hazard_perception_v0.evaluation import maximum_cardinality_min_distance_matching


def _rotate(quaternion, point):
    x, y, z, w = [float(value) for value in quaternion]
    px, py, pz = [float(value) for value in point]
    uv = (
        y * pz - z * py,
        z * px - x * pz,
        x * py - y * px,
    )
    uuv = (
        y * uv[2] - z * uv[1],
        z * uv[0] - x * uv[2],
        x * uv[1] - y * uv[0],
    )
    return [
        point[index] + 2.0 * (w * uv[index] + uuv[index])
        for index in range(3)
    ]


def _prediction_xyz(item, payload):
    if "xyz" in item:
        point = [float(value) for value in item["xyz"]]
    else:
        point = [float(item[axis]) for axis in ("x", "y", "z")]
    if payload.get("frame_id") != "start_frame":
        return point
    transform = payload.get("start_frame_transform")
    if not transform:
        raise ValueError("start-frame predictions have no persisted transform")
    translation = transform["translation"]
    quaternion = transform["quaternion_xyzw"]
    rotated = _rotate(
        (quaternion["x"], quaternion["y"], quaternion["z"], quaternion["w"]),
        point,
    )
    return [
        rotated[0] + float(translation["x"]),
        rotated[1] + float(translation["y"]),
        rotated[2] + float(translation["z"]),
    ]


def _distance(left, right):
    return math.sqrt(sum(
        (float(left[index]) - float(right[index])) ** 2
        for index in range(3)
    ))


def _match(predictions, truth, threshold):
    return maximum_cardinality_min_distance_matching(predictions, truth, threshold)


def evaluate(prediction_path, truth_path, output_path=None, threshold=0.75):
    with open(prediction_path, encoding="utf-8") as source:
        prediction_payload = json.load(source)
    with open(truth_path, encoding="utf-8") as source:
        truth_payload = json.load(source)
    predictions = [
        _prediction_xyz(item, prediction_payload)
        for item in prediction_payload.get("hazards", [])
    ]
    truth = [
        list(item["position"])
        for item in truth_payload.get("danger_sources", [])
    ]
    matches = _match(predictions, truth, threshold)
    matched_predictions = {item["prediction_index"] for item in matches}
    matched_truth = {item["truth_index"] for item in matches}
    errors = [item["error_m"] for item in matches]
    report = {
        "prediction_frame": prediction_payload.get("frame_id", "unknown"),
        "truth_frame": "world",
        "threshold_m": float(threshold),
        "truth_count": len(truth),
        "prediction_count": len(predictions),
        "TP": len(matches),
        "FP": len(predictions) - len(matched_predictions),
        "FN": len(truth) - len(matched_truth),
        "precision": (
            len(matches) / float(len(predictions))
            if predictions else 0.0
        ),
        "recall": (
            len(matches) / float(len(truth))
            if truth else 1.0
        ),
        "f1": (
            2.0 * len(matches) / float(len(predictions) + len(truth))
            if predictions or truth else 1.0
        ),
        "mean_error_m": sum(errors) / len(errors) if errors else None,
        "max_error_m": max(errors) if errors else None,
        "matches": matches,
        "passed": len(matches) == len(truth) == len(predictions),
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
    print(json.dumps(
        evaluate(
            args.predictions,
            args.truth,
            args.output or None,
            args.threshold,
        ),
        indent=2,
        sort_keys=True,
    ))


if __name__ == "__main__":
    main()
