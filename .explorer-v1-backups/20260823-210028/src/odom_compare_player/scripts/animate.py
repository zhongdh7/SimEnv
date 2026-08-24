#!/usr/bin/env python3
"""Animate the recorded GT vs FAST-LIO trajectories in RViz.

Reads the CSV, aligns FAST-LIO onto ground truth exactly like ``visualizer.py``
(same ``~align_sec``), then steps through the samples in real time, publishing:

  * ``/gt_path_anim`` / ``/lio_path_anim`` -- growing Marker LINE_STRIP trails
  * ``/gt_pose`` / ``/lio_pose``           -- the current PoseStamped pose

All in the single ``~frame`` (default ``world``) so RViz needs no TF tree.
Lets you watch the drift accumulate over the course of the run.
"""

from __future__ import print_function

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import rospy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

import odom_compare_lib as lib


def _make_line_strip(frame, ns, color, scale):
    m = Marker()
    m.header = Header(frame_id=frame, stamp=rospy.Time.now())
    m.ns = ns
    m.id = 0
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = scale
    m.color.r, m.color.g, m.color.b, m.color.a = color
    return m


def _pose_stamped(frame, p, q):
    ps = PoseStamped()
    ps.header = Header(frame_id=frame, stamp=rospy.Time.now())
    ps.pose.position = Point(*p)
    ps.pose.orientation = Quaternion(*q)
    return ps


def main():
    rospy.init_node("odom_compare_animate")
    csv_path = os.path.expanduser(
        rospy.get_param("~csv_path", "~/odom_compare/odom_compare.csv"))
    align_sec = rospy.get_param("~align_sec", 5.0)
    frame = rospy.get_param("~frame", "world")
    rate = rospy.get_param("~rate", 10.0)
    loop = rospy.get_param("~loop", True)

    data = lib.load_csv(csv_path)
    _, _, aligned = lib.align_trajectory(data, align_sec)
    gt = data["gt"]
    lio = aligned
    n = len(gt)

    pub_gt_trail = rospy.Publisher("/gt_path_anim", Marker, queue_size=1)
    pub_lio_trail = rospy.Publisher("/lio_path_anim", Marker, queue_size=1)
    pub_gt_pose = rospy.Publisher("/gt_pose", PoseStamped, queue_size=1)
    pub_lio_pose = rospy.Publisher("/lio_pose", PoseStamped, queue_size=1)

    gt_trail = _make_line_strip(frame, "gt_anim", (0.0, 1.0, 0.0, 1.0), 0.12)
    lio_trail = _make_line_strip(frame, "lio_anim", (1.0, 0.0, 0.0, 1.0), 0.12)

    while not rospy.is_shutdown():
        for i in range(n):
            if rospy.is_shutdown():
                return
            gt_trail.points.append(Point(*gt[i, 0:3]))
            lio_trail.points.append(Point(*lio[i, 0:3]))
            gt_trail.header.stamp = rospy.Time.now()
            lio_trail.header.stamp = rospy.Time.now()
            pub_gt_trail.publish(gt_trail)
            pub_lio_trail.publish(lio_trail)
            pub_gt_pose.publish(_pose_stamped(frame, gt[i, 0:3], gt[i, 3:7]))
            pub_lio_pose.publish(_pose_stamped(frame, lio[i, 0:3], lio[i, 3:7]))

            # Advance in wall-clock time to match the recorded cadence.
            if i + 1 < n:
                dt = data["t"][i + 1] - data["t"][i]
            else:
                dt = 1.0 / rate
            if dt > 0:
                time.sleep(min(dt, 1.0))

        rospy.loginfo("reached end of trajectory%s", "" if loop else "; stopping")
        if not loop:
            break


if __name__ == "__main__":
    main()
