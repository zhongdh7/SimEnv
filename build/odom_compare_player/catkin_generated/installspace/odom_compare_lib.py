#!/usr/bin/env python3
"""Shared helpers for the odom-compare offline player/analysis scripts.

Kept free of any ROS import so ``analyze.py`` can run standalone (python3
without a ROS master).  Depends on numpy only.
"""

from __future__ import print_function

import csv
import math

import numpy as np


def load_csv(path):
    """Load a recorder CSV into a dict of numpy arrays.

    Returns {"t": (N,), "gt": (N, 7), "lio": (N, 7)} where each pose row is
    [x, y, z, qx, qy, qz, qw].
    """
    t = []
    gt = []
    lio = []
    with open(path, "r") as f:
        for row in csv.DictReader(f):
            t.append(float(row["t"]))
            gt.append([
                float(row["gt_x"]), float(row["gt_y"]), float(row["gt_z"]),
                float(row["gt_qx"]), float(row["gt_qy"]),
                float(row["gt_qz"]), float(row["gt_qw"]),
            ])
            lio.append([
                float(row["lio_x"]), float(row["lio_y"]), float(row["lio_z"]),
                float(row["lio_qx"]), float(row["lio_qy"]),
                float(row["lio_qz"]), float(row["lio_qw"]),
            ])
    return {
        "t": np.asarray(t, dtype=float),
        "gt": np.asarray(gt, dtype=float),
        "lio": np.asarray(lio, dtype=float),
    }


def quat_to_matrix(q):
    """Quaternion [x, y, z, w] -> 3x3 rotation matrix."""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0:
        n = 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(R):
    """3x3 rotation matrix -> quaternion [x, y, z, w]."""
    R = np.asarray(R, dtype=float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def yaw_from_quat(q):
    """Yaw angle (rad) of quaternion [x, y, z, w]."""
    R = quat_to_matrix(q)
    return math.atan2(R[1, 0], R[0, 0])


def align_rigid(src, dst):
    """Rigid (rotation + translation, no scale) alignment: R @ src + t ~ dst.

    src, dst: (N, 3) numpy arrays of corresponding points.
    Returns R (3x3), t (3,).
    """
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    src_c = src - src.mean(axis=0)
    dst_c = dst - dst.mean(axis=0)
    H = src_c.T @ dst_c
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = dst.mean(axis=0) - R @ src.mean(axis=0)
    return R, t


def align_trajectory(data, align_sec=5.0):
    """Align LIO onto GT using the first ``align_sec`` seconds.

    The two world frames are both anchored at the initial pose, so this only
    removes a small constant offset and does NOT absorb the drift we want to
    measure (which is why we align on the head of the run, not the whole run).

    Returns (R, t, aligned_lio) where aligned_lio is (N, 7) [x y z qx qy qz qw]
    expressed in the GT frame.
    """
    t0 = data["t"][0]
    mask = data["t"] <= t0 + align_sec
    if mask.sum() < 3:
        mask = np.ones(len(data["t"]), dtype=bool)
    R, t = align_rigid(data["lio"][mask, 0:3], data["gt"][mask, 0:3])

    aligned = np.empty_like(data["lio"])
    for i in range(len(data["lio"])):
        ap = R @ data["lio"][i, 0:3] + t
        aq = matrix_to_quat(R @ quat_to_matrix(data["lio"][i, 3:7]))
        aligned[i] = np.concatenate([ap, aq])
    return R, t, aligned


def metrics(data, aligned):
    """Position-error metrics of the aligned LIO vs GT. Returns a dict."""
    err = np.linalg.norm(aligned[:, 0:3] - data["gt"][:, 0:3], axis=1)
    return {
        "ate_rmse": float(np.sqrt(np.mean(err ** 2))),
        "mean": float(np.mean(err)),
        "max": float(np.max(err)),
        "end": float(err[-1]),
        "err": err,
    }
