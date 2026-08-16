#!/usr/bin/env python3
"""Publish the recorded GT + FAST-LIO trajectories as an overlaid RViz overlay.

Reads the CSV written by ``recorder.py``, rigidly aligns FAST-LIO onto ground
truth using the first ``~align_sec`` seconds (both world frames are anchored at
the initial pose, so this only removes a small constant offset and does NOT
absorb the drift we want to see), then publishes on ``/odom_compare_markers``:

  * ns "gt"        Marker LINE_STRIP (green)  -- ground-truth trajectory
  * ns "lio"       Marker LINE_STRIP (red)    -- aligned FAST-LIO trajectory
  * ns "deviation" Marker LINE_LIST  (blue)   -- per-sample GT->LIO offset

Everything is published in a single ``~frame`` (default ``world``) so RViz needs
no TF tree; set RViz Fixed Frame to the same value.  Also prints drift metrics
(ATE RMSE, mean/max/end drift, yaw drift).
"""

from __future__ import print_function

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import numpy as np
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

import odom_compare_lib as lib


def _line_strip(frame, ns, color, scale, pts):
    m = Marker()
    m.header = Header(frame_id=frame, stamp=rospy.Time.now())
    m.ns = ns
    m.id = 0
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = scale
    m.color.r, m.color.g, m.color.b, m.color.a = color
    m.points = [Point(*p) for p in pts]
    return m


def _line_list(frame, ns, color, scale, pts):
    m = Marker()
    m.header = Header(frame_id=frame, stamp=rospy.Time.now())
    m.ns = ns
    m.id = 0
    m.type = Marker.LINE_LIST
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = scale
    m.color.r, m.color.g, m.color.b, m.color.a = color
    m.points = [Point(*p) for p in pts]
    return m


def main():
    rospy.init_node("odom_compare_visualizer")
    csv_path = os.path.expanduser(
        rospy.get_param("~csv_path", "~/odom_compare/odom_compare.csv"))
    align_sec = rospy.get_param("~align_sec", 5.0)
    frame = rospy.get_param("~frame", "world")

    data = lib.load_csv(csv_path)
    _, _, aligned = lib.align_trajectory(data, align_sec)

    gt_pts = data["gt"][:, 0:3]
    lio_pts = aligned[:, 0:3]

    pub = rospy.Publisher("/odom_compare_markers", Marker, queue_size=10, latch=True)
    pub.publish(_line_strip(frame, "gt", (0.0, 1.0, 0.0, 1.0), 0.10, gt_pts))
    pub.publish(_line_strip(frame, "lio", (1.0, 0.0, 0.0, 1.0), 0.10, lio_pts))

    # Deviation: connecting segments, thinned so the plot stays readable.
    stride = max(1, int(len(gt_pts) / 500))
    dev_pts = []
    for i in range(0, len(gt_pts), stride):
        dev_pts.append(gt_pts[i])
        dev_pts.append(lio_pts[i])
    pub.publish(_line_list(frame, "deviation", (0.3, 0.3, 1.0, 0.8), 0.03, dev_pts))

    # Drift metrics.
    met = lib.metrics(data, aligned)
    gt_yaw = np.array([lib.yaw_from_quat(d[3:7]) for d in data["gt"]])
    lio_yaw = np.array([lib.yaw_from_quat(d[3:7]) for d in aligned])
    yaw_err = (lio_yaw - gt_yaw + math.pi) % (2 * math.pi) - math.pi

    rospy.loginfo("published markers in frame '%s' (set RViz Fixed Frame to '%s')",
                  frame, frame)
    rospy.loginfo(
        "samples=%d  ATE RMSE=%.3f m  mean=%.3f m  max=%.3f m  end=%.3f m",
        len(gt_pts), met["ate_rmse"], met["mean"], met["max"], met["end"])
    rospy.loginfo("yaw drift: final=%.2f deg  max_abs=%.2f deg",
                  math.degrees(yaw_err[-1]), math.degrees(np.max(np.abs(yaw_err))))
    rospy.loginfo("spinning to keep latched topics alive (Ctrl-C to quit)")
    rospy.spin()


if __name__ == "__main__":
    main()
