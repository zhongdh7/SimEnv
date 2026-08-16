#!/usr/bin/env python3
"""Record ground-truth and FAST-LIO odometry side by side.

Subscribes to the ground-truth odometry (``/Odometry_gazebo``, frame ``odom``
-> ``base``) and the FAST-LIO odometry (``/Odometry``, frame ``camera_init``
-> ``body``) and writes two artefacts into one output directory:

  * ``odom_compare.csv`` -- time-aligned pose pairs (one row per FAST-LIO
    sample), consumed by the offline player/analysis scripts;
  * ``odom_compare.bag``  -- a *compact* rosbag containing ONLY these two
    odometry topics.  Ground truth arrives at ~500 Hz, far more than the ~10 Hz
    FAST-LIO stream needs, so it is throttled to ``~bag_gt_max_hz`` inside the
    bag.  Never records /scan or point clouds, so the bag stays small.

Both messages are ``nav_msgs/Odometry`` whose pose is the robot base expressed
in the source world frame.  The two world frames (``odom`` and ``camera_init``)
are both anchored at the initial robot pose, so they coincide up to the unknown
LIO drift; the player estimates the small constant offset with a rigid
alignment over the first few seconds.
"""

from __future__ import print_function

import csv
import math
import os
from collections import deque

import rospy
import rosbag
from nav_msgs.msg import Odometry


def _finite_odom(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))


class OdomCompareRecorder(object):
    def __init__(self):
        self.gt_topic = rospy.get_param("~gt_topic", "/Odometry_gazebo")
        self.lio_topic = rospy.get_param("~lio_topic", "/Odometry")
        out_dir = os.path.expanduser(
            rospy.get_param("~output_dir", "~/odom_compare"))
        csv_name = rospy.get_param("~csv_name", "odom_compare.csv")
        bag_name = rospy.get_param("~bag_name", "odom_compare.bag")
        self.max_sync_sec = rospy.get_param("~max_sync_sec", 0.05)
        self.bag_gt_max_hz = rospy.get_param("~bag_gt_max_hz", 20.0)

        os.makedirs(out_dir, exist_ok=True)
        self.csv_path = os.path.join(out_dir, csv_name)
        self.bag_path = os.path.join(out_dir, bag_name)

        # Ground-truth buffer for nearest-in-time pairing (GT arrives at ~500 Hz).
        self.gt_buf = deque()  # (stamp_sec, msg)
        self._last_bag_gt_t = -1e9
        self._n_gt = 0
        self._n_lio = 0
        self._n_csv = 0
        self._skipped = 0

        self._csv = open(self.csv_path, "w", newline="")
        self._writer = csv.writer(self._csv)
        self._writer.writerow([
            "t",
            "gt_x", "gt_y", "gt_z", "gt_qx", "gt_qy", "gt_qz", "gt_qw",
            "lio_x", "lio_y", "lio_z", "lio_qx", "lio_qy", "lio_qz", "lio_qw",
        ])

        self._bag = rosbag.Bag(self.bag_path, "w")

        rospy.Subscriber(self.gt_topic, Odometry, self._gt_cb, queue_size=2000)
        rospy.Subscriber(self.lio_topic, Odometry, self._lio_cb, queue_size=200)

        rospy.on_shutdown(self._finish)
        rospy.loginfo(
            "recording GT '%s' + LIO '%s' -> %s (Ctrl-C to stop and save)",
            self.gt_topic, self.lio_topic, out_dir,
        )

    # -- subscribers -------------------------------------------------------
    def _gt_cb(self, msg):
        self._n_gt += 1
        if not _finite_odom(msg):
            return
        t = msg.header.stamp.to_sec()
        self.gt_buf.append((t, msg))
        # Drop samples older than 2 s; LIO runs at ~10 Hz so the nearest GT is
        # always far closer than that.
        while self.gt_buf and self.gt_buf[0][0] < t - 2.0:
            self.gt_buf.popleft()
        self._write_bag_gt(msg)

    def _lio_cb(self, msg):
        self._n_lio += 1
        if not _finite_odom(msg):
            return
        t = msg.header.stamp.to_sec()
        self._bag.write(self.lio_topic, msg, t=msg.header.stamp)

        # Pair with the nearest ground-truth sample within tolerance.
        best = None
        best_dt = self.max_sync_sec
        for gt_t, gt_msg in self.gt_buf:
            dt = abs(gt_t - t)
            if dt < best_dt:
                best_dt = dt
                best = gt_msg
        if best is None:
            self._skipped += 1
            return

        gp = best.pose.pose.position
        gq = best.pose.pose.orientation
        lp = msg.pose.pose.position
        lq = msg.pose.pose.orientation
        self._writer.writerow([
            t,
            gp.x, gp.y, gp.z, gq.x, gq.y, gq.z, gq.w,
            lp.x, lp.y, lp.z, lq.x, lq.y, lq.z, lq.w,
        ])
        self._n_csv += 1

    def _write_bag_gt(self, msg):
        # Throttle GT in the bag: 500 Hz -> ~20 Hz is plenty for replay next to
        # the ~10 Hz LIO stream and keeps the bag small.
        t = msg.header.stamp.to_sec()
        if t - self._last_bag_gt_t >= 1.0 / self.bag_gt_max_hz:
            self._bag.write(self.gt_topic, msg, t=msg.header.stamp)
            self._last_bag_gt_t = t

    # -- shutdown ----------------------------------------------------------
    def _finish(self):
        self._csv.flush()
        self._csv.close()
        try:
            self._bag.close()
        except Exception:
            pass
        rospy.loginfo(
            "recorded %d GT + %d LIO msgs, %d CSV rows (skipped %d unmatched) -> %s",
            self._n_gt, self._n_lio, self._n_csv, self._skipped, self.csv_path,
        )


if __name__ == "__main__":
    rospy.init_node("odom_compare_recorder")
    OdomCompareRecorder()
    rospy.spin()
