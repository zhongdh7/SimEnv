#!/usr/bin/env python3
"""Offline drift analysis: ATE / RPE / yaw metrics + matplotlib plots.

Runs standalone (no ROS master needed):

    python3 analyze.py --csv ~/odom_compare/odom_compare.csv

Produces a PNG (default ``<csv>_analysis.png``) with four panels:
  1) 2-D top-down overlay (GT vs aligned FAST-LIO);
  2) position error over time;
  3) yaw error over time;
  4) drift vs. cumulative distance.
"""

from __future__ import print_function

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import odom_compare_lib as lib


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--csv", required=True, help="recorder CSV path")
    ap.add_argument("--out", default=None, help="output PNG path")
    ap.add_argument("--align-sec", type=float, default=5.0,
                    help="head-of-run alignment window (s)")
    ap.add_argument("--rpe-delta", type=float, default=10.0,
                    help="RPE distance window (m)")
    args = ap.parse_args()

    data = lib.load_csv(args.csv)
    _, _, aligned = lib.align_trajectory(data, args.align_sec)
    gt = data["gt"]
    lio = aligned
    t = data["t"] - data["t"][0]
    err = np.linalg.norm(lio[:, 0:3] - gt[:, 0:3], axis=1)

    gt_yaw = np.array([lib.yaw_from_quat(d[3:7]) for d in gt])
    lio_yaw = np.array([lib.yaw_from_quat(d[3:7]) for d in lio])
    yaw_err = (lio_yaw - gt_yaw + math.pi) % (2 * math.pi) - math.pi

    # Cumulative distance along the ground-truth trajectory.
    d = np.zeros(len(gt))
    d[1:] = np.cumsum(np.linalg.norm(np.diff(gt[:, 0:3], axis=0), axis=1))

    # Simplified chord-distance RPE over a ~rpe_delta m travel window.
    rpe = []
    rpe_d = []
    i = 0
    for j in range(len(gt)):
        while i < j and d[j] - d[i] > args.rpe_delta:
            i += 1
        if i > 0 and d[j] - d[i] > 0.5 * args.rpe_delta:
            rpe.append(abs(np.linalg.norm(lio[j, 0:3] - lio[i, 0:3])
                           - np.linalg.norm(gt[j, 0:3] - gt[i, 0:3])))
            rpe_d.append(d[j])
    rpe = np.array(rpe)

    fig, axes = plt.subplots(2, 2, figsize=(12, 9))

    axes[0, 0].plot(gt[:, 0], gt[:, 1], "g-", lw=1.2, label="ground truth")
    axes[0, 0].plot(lio[:, 0], lio[:, 1], "r-", lw=1.0, label="FAST-LIO (aligned)")
    axes[0, 0].set_aspect("equal")
    axes[0, 0].legend()
    axes[0, 0].set_title("2-D trajectory")
    axes[0, 0].set_xlabel("x (m)")
    axes[0, 0].set_ylabel("y (m)")

    axes[0, 1].plot(t, err)
    axes[0, 1].set_title("Position error over time")
    axes[0, 1].set_xlabel("time (s)")
    axes[0, 1].set_ylabel("error (m)")

    axes[1, 0].plot(t, np.degrees(yaw_err))
    axes[1, 0].set_title("Yaw error over time")
    axes[1, 0].set_xlabel("time (s)")
    axes[1, 0].set_ylabel("yaw error (deg)")

    axes[1, 1].plot(d, err)
    axes[1, 1].set_title("Drift vs. cumulative distance")
    axes[1, 1].set_xlabel("distance (m)")
    axes[1, 1].set_ylabel("error (m)")

    fig.suptitle(
        "ATE RMSE=%.3f m  mean=%.3f m  max=%.3f m  end=%.3f m"
        % (float(np.sqrt(np.mean(err ** 2))), float(np.mean(err)),
           float(np.max(err)), float(err[-1])))
    fig.tight_layout()
    out = args.out or os.path.splitext(args.csv)[0] + "_analysis.png"
    fig.savefig(out, dpi=120)
    print("wrote", out)
    if len(rpe):
        print("RPE over ~%.0f m window: mean=%.3f m  max=%.3f m"
              % (args.rpe_delta, float(rpe.mean()), float(rpe.max())))
    print("final yaw drift: %.2f deg" % math.degrees(yaw_err[-1]))


if __name__ == "__main__":
    main()
