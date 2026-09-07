#!/usr/bin/env python3
"""Unified offline evaluation for one finalized hazard run.

Only this post-run evaluator may read ``danger_truth.json``.  No runtime
pipeline, pose bridge, tracker or finalize tool imports this module or consumes
simulator truth, scene manifests, layout metadata or Gazebo model truth.
"""

import argparse
import json
import math
import os
import sys

try:
    from hazard_perception_v0.evaluation import (
        error_statistics,
        maximum_cardinality_min_distance_matching,
        transform_point,
    )
except ImportError:  # Direct execution from an unbuilt source checkout.
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
    from hazard_perception_v0.evaluation import (  # noqa: E402
        error_statistics,
        maximum_cardinality_min_distance_matching,
        transform_point,
    )


def _xyz(item):
    values = item.get("xyz")
    if values is None:
        values = [item[axis] for axis in ("x", "y", "z")]
    values = [float(value) for value in values]
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError("XYZ must contain exactly three finite values")
    return values


def _load_predictions(payload):
    prediction_frame = str(payload.get("frame_id", "unknown"))
    transform = payload.get("start_frame_transform")
    predictions = []
    for index, item in enumerate(payload.get("hazards", [])):
        raw_xyz = _xyz(item)
        if prediction_frame == "start_frame":
            if not transform:
                raise ValueError("start_frame predictions have no persisted transform")
            evaluated_xyz = transform_point(transform, raw_xyz)
        else:
            evaluated_xyz = raw_xyz
        predictions.append({
            "index": index,
            "id": item.get("id", index),
            "raw_xyz": raw_xyz,
            "xyz": evaluated_xyz,
            "confidence": item.get("confidence"),
            "localization_source": item.get("localization_source", "unknown"),
        })
    return prediction_frame, predictions


def _load_truth(payload):
    truth = []
    for index, item in enumerate(payload.get("danger_sources", [])):
        xyz = [float(value) for value in item["position"]]
        if len(xyz) != 3 or not all(math.isfinite(value) for value in xyz):
            raise ValueError("truth position must contain exactly three finite values")
        truth.append({
            "index": index,
            "id": item.get("id", index),
            "xyz": xyz,
        })
    return truth


def evaluate(prediction_path, truth_path, output_path=None, threshold=0.75):
    """Evaluate a finalized sensor result against post-run simulator truth."""
    with open(prediction_path, encoding="utf-8") as source:
        prediction_payload = json.load(source)
    # This is the sole truth read.  It occurs only in this offline evaluator.
    with open(truth_path, encoding="utf-8") as source:
        truth_payload = json.load(source)

    prediction_frame, predictions = _load_predictions(prediction_payload)
    truth = _load_truth(truth_payload)
    raw_matches = maximum_cardinality_min_distance_matching(
        [item["xyz"] for item in predictions],
        [item["xyz"] for item in truth],
        threshold,
    )
    matched_prediction_indices = {item["prediction_index"] for item in raw_matches}
    matched_truth_indices = {item["truth_index"] for item in raw_matches}
    matches = []
    for match in raw_matches:
        prediction = predictions[match["prediction_index"]]
        target = truth[match["truth_index"]]
        matches.append({
            "prediction_index": prediction["index"],
            "prediction_id": prediction["id"],
            "truth_index": target["index"],
            "truth_id": target["id"],
            "pred_xyz": prediction["xyz"],
            "truth_xyz": target["xyz"],
            "error_m": match["error_m"],
        })
    false_positives = [
        {
            "prediction_index": item["index"],
            "prediction_id": item["id"],
            "xyz": item["xyz"],
            "confidence": item["confidence"],
            "localization_source": item["localization_source"],
        }
        for index, item in enumerate(predictions)
        if index not in matched_prediction_indices
    ]
    false_negatives = [
        {
            "truth_index": item["index"],
            "truth_id": item["id"],
            "truth_xyz": item["xyz"],
        }
        for index, item in enumerate(truth)
        if index not in matched_truth_indices
    ]

    tp = len(matches)
    fp = len(false_positives)
    fn = len(false_negatives)
    prediction_count = len(predictions)
    gt_count = len(truth)
    precision = tp / float(tp + fp) if tp + fp else 0.0
    recall = tp / float(tp + fn) if tp + fn else 1.0
    false_alarm_ratio = fp / float(tp + fp) if tp + fp else 0.0
    miss_rate = fn / float(tp + fn) if tp + fn else 0.0
    f1 = (
        2.0 * precision * recall / (precision + recall)
        if precision + recall else (1.0 if not prediction_count and not gt_count else 0.0)
    )
    transform = prediction_payload.get("start_frame_transform")
    truth_frame = str(truth_payload.get("frame_id", "world"))
    if prediction_frame == "start_frame" and transform:
        truth_frame = str(transform.get("parent_frame", truth_frame))
    report = {
        "schema": "hazard_evaluation_v1",
        "prediction_path": os.path.abspath(prediction_path),
        "truth_path": os.path.abspath(truth_path),
        "prediction_frame": prediction_frame,
        "truth_frame": truth_frame,
        "threshold_m": float(threshold),
        "gt_count": gt_count,
        "prediction_count": prediction_count,
        "TP": tp,
        "FP": fp,
        "FN": fn,
        "detection_rate": recall,
        "recall": recall,
        "precision": precision,
        "false_alarm_ratio": false_alarm_ratio,
        "miss_rate": miss_rate,
        "f1": f1,
        "matches": matches,
        "false_positives": false_positives,
        "false_negatives": false_negatives,
    }
    report.update(error_statistics(item["error_m"] for item in matches))
    if output_path:
        output_path = os.path.abspath(os.path.expanduser(output_path))
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        temporary_path = output_path + ".tmp"
        with open(temporary_path, "w", encoding="utf-8") as output:
            json.dump(report, output, ensure_ascii=False, indent=2, sort_keys=True)
            output.write("\n")
        os.replace(temporary_path, output_path)
    return report


def _format_xyz(values):
    return "({:.3f}, {:.3f}, {:.3f})".format(*values)


def human_report(report):
    def percent(value):
        return "{:7.2f} %".format(100.0 * value)

    def error(value):
        return "n/a" if value is None else "{:.3f} m".format(value)

    lines = [
        "========== Hazard Evaluation ==========", "",
        "Ground Truth:       {}".format(report["gt_count"]),
        "Predictions:        {}".format(report["prediction_count"]), "",
        "TP:                 {}".format(report["TP"]),
        "FP:                 {}".format(report["FP"]),
        "FN:                 {}".format(report["FN"]), "",
        "Detection Rate:     {}".format(percent(report["detection_rate"])),
        "Recall:             {}".format(percent(report["recall"])),
        "Precision:          {}".format(percent(report["precision"])),
        "False Alarm Ratio:  {}".format(percent(report["false_alarm_ratio"])),
        "Miss Rate:          {}".format(percent(report["miss_rate"])),
        "F1:                 {}".format(percent(report["f1"])), "",
        "Localization:",
        "Mean Error:         {}".format(error(report["mean_error_m"])),
        "Median Error:       {}".format(error(report["median_error_m"])),
        "P95 Error:          {}".format(error(report["p95_error_m"])),
        "Max Error:          {}".format(error(report["max_error_m"])), "",
        "---------- Matches ----------",
    ]
    lines.extend(
        "prediction {} (index {}) {} -> truth {} (index {}) {} error={:.3f} m".format(
            item["prediction_id"], item["prediction_index"], _format_xyz(item["pred_xyz"]),
            item["truth_id"], item["truth_index"], _format_xyz(item["truth_xyz"]),
            item["error_m"],
        ) for item in report["matches"]
    )
    if not report["matches"]:
        lines.append("(none)")
    lines.extend(["", "---------- False Positives ----------"])
    lines.extend(
        "prediction {} (index {}) {} confidence={} source={}".format(
            item["prediction_id"], item["prediction_index"], _format_xyz(item["xyz"]),
            item["confidence"], item["localization_source"],
        ) for item in report["false_positives"]
    )
    if not report["false_positives"]:
        lines.append("(none)")
    lines.extend(["", "---------- Missed Hazards ----------"])
    lines.extend(
        "truth {} (index {}) {}".format(
            item["truth_id"], item["truth_index"], _format_xyz(item["truth_xyz"])
        ) for item in report["false_negatives"]
    )
    if not report["false_negatives"]:
        lines.append("(none)")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("predictions", help="finalized hazards_*.json")
    parser.add_argument("truth", help="post-run SimEnv danger_truth.json")
    parser.add_argument("--threshold", type=float, default=0.75)
    parser.add_argument("--output", default="", help="optional JSON report path")
    args = parser.parse_args()
    try:
        report = evaluate(
            os.path.expanduser(args.predictions), os.path.expanduser(args.truth),
            os.path.expanduser(args.output) if args.output else None, args.threshold,
        )
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        print("hazard evaluation failed: {}".format(error), file=sys.stderr)
        return 2
    print(human_report(report))
    if args.output:
        print("\nJSON report: {}".format(os.path.abspath(os.path.expanduser(args.output))))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
