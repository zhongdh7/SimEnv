#!/usr/bin/env python3
"""Bridge FAST-LIO2 odometry into the navigation node's TF expectations.

The navigation node (``competition_navigation_node.py``, copied unchanged) reads
its robot pose from a ``nav_msgs/Odometry`` message and transforms every scan
point via ``lookupTransform(map_frame, scan_frame)``.  With Gazebo ground truth
that works because ``state_from_gazebo`` broadcasts ``odom -> base`` and
``robot_state_publisher`` broadcasts ``base -> laser_livox``.

FAST-LIO2 instead publishes:

  * ``/Odometry`` in the ``camera_init`` frame with child ``body``, and
  * TF ``camera_init -> body`` only.

``body`` is the same physical link as the robot's ``base`` (the IMU/trunk), but
``camera_init`` is a DIFFERENT frame from the ground-truth ``odom`` frame: it is
the robot's own pose at FAST-LIO2 start-up (identity yaw), whereas ``odom`` is
the Gazebo world frame in which the building is axis-aligned (the main entrance
sits at world x=0 and the corridor runs along +Y, and the robot spawns facing
+Y, i.e. yaw = pi/2).

The navigation logic is *not* frame-agnostic: its door/room/corridor/stairs
heuristics assume the map frame is aligned with the building axes.  If we feed
it ``camera_init`` directly, the whole map appears rotated ~90 deg and the robot
drives off sideways instead of entering the main gate.

This node therefore re-anchors FAST-LIO2's estimate into the building-aligned
``odom`` frame using a ONE-TIME initial alignment.  At start-up it captures the
robot's ground-truth pose (``/Odometry_gazebo``) together with FAST-LIO2's first
pose and derives the constant transform ``odom <- camera_init``; that transform
is then applied to every subsequent FAST-LIO2 pose.  Ground truth is used only
for this single initial anchor (the equivalent of "the robot knows where it
starts"), never for continuous localisation -- after anchoring, the estimate is
purely FAST-LIO2's.  It then:

  1. publishes the transformed pose on ``/Odometry_fastlio`` (frame ``odom``),
  2. broadcasts ``odom -> laser_livox_fastlio`` from that pose plus the known
     lidar extrinsic, and
  3. re-publishes ``/scan`` under the re-labelled frame ``laser_livox_fastlio``.

The navigation node is launched with ``map_frame=odom``,
``scan_topic=/scan_fastlio`` and ``odom_topic=/Odometry_fastlio``; its TF
lookups and sensor-origin logic behave exactly as the ground-truth version.
"""

from __future__ import print_function

import math
import threading

import numpy as np
import rospy
import tf
import tf.transformations as tft
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud


def _finite(vals):
    return all(math.isfinite(float(v)) for v in vals)


def _rot_matrix(q):
    return tft.quaternion_matrix(q)[0:3, 0:3]


class ScanTfRelay(object):
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.gt_odom_topic = rospy.get_param("~gt_odom_topic", "/Odometry_gazebo")
        self.out_odom_topic = rospy.get_param("~out_odom_topic", "/Odometry_fastlio")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.out_scan_topic = rospy.get_param("~out_scan_topic", "/scan_fastlio")
        self.map_frame = rospy.get_param("~map_frame", "odom")
        self.laser_frame = rospy.get_param("~laser_frame", "laser_livox_fastlio")

        # Lidar -> IMU/body extrinsic (matches fastlio2_sim.yaml):
        #   p_body = R_bl * p_lidar + t_bl,  R_bl = Ry(+45 deg), t_bl = lever arm.
        t_bl = rospy.get_param("~extrinsic_T", [0.2, 0.0, 0.08])
        rpy_bl = rospy.get_param(
            "~extrinsic_rpy", [0.0, math.pi / 4.0, 0.0]
        )
        self.t_bl = (float(t_bl[0]), float(t_bl[1]), float(t_bl[2]))
        self.q_bl = tft.quaternion_from_euler(
            float(rpy_bl[0]), float(rpy_bl[1]), float(rpy_bl[2])
        )

        self._lock = threading.Lock()
        self._br = tf.TransformBroadcaster()

        # One-time alignment ``odom <- camera_init``: derived from the first
        # ground-truth pose and FAST-LIO2's first pose.  None until ready.
        self._aligned = False
        self._q_oc = None  # rotation camera_init -> odom
        self._t_oc = None  # translation camera_init -> odom
        self._latest_gt = None  # (t, q) of base in odom, newest /Odometry_gazebo

        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=5)
        rospy.Subscriber(
            self.gt_odom_topic, Odometry, self._gt_cb, queue_size=5,
        )
        rospy.Subscriber(self.scan_topic, PointCloud, self._scan_cb, queue_size=1)
        self._odom_pub = rospy.Publisher(
            self.out_odom_topic, Odometry, queue_size=5,
        )
        self._scan_pub = rospy.Publisher(
            self.out_scan_topic, PointCloud, queue_size=1,
        )

        rospy.loginfo(
            "scan_tf_relay: odom=%s -> %s (map=%s, laser=%s, t_bl=%s)",
            self.odom_topic, self.out_odom_topic,
            self.map_frame, self.laser_frame, self.t_bl,
        )
        rospy.loginfo(
            "scan_tf_relay: initial anchor from %s (used once at start-up only)",
            self.gt_odom_topic,
        )

    def _gt_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        if not _finite((p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            return
        with self._lock:
            self._latest_gt = (
                (p.x, p.y, p.z),
                (q.x, q.y, q.z, q.w),
            )

    def _try_lock_alignment(self, q_cb0, t_cb0):
        """Derive and lock ``odom <- camera_init`` from the first pose pair."""
        with self._lock:
            if self._aligned:
                return
            gt = self._latest_gt
            if gt is None:
                rospy.logwarn_throttle(
                    5.0,
                    "scan_tf_relay: waiting for %s to anchor the initial pose",
                    self.gt_odom_topic,
                )
                return
            t_ob0, q_ob0 = gt

            # R_oc = R_ob(0) * R_cb(0)^T,  t_oc = t_ob(0) - R_oc * t_cb(0).
            q_oc = tft.quaternion_multiply(q_ob0, tft.quaternion_inverse(q_cb0))
            R_oc = _rot_matrix(q_oc)
            t_cb0_arr = np.asarray(t_cb0, dtype=np.float64)
            t_oc = tuple(np.asarray(t_ob0, dtype=np.float64) - R_oc.dot(t_cb0_arr))

            self._q_oc = q_oc
            self._t_oc = t_oc
            self._aligned = True

            yaw = tft.euler_from_quaternion(q_oc)[2]
            rospy.loginfo(
                "scan_tf_relay: alignment locked "
                "t_oc=(%.3f, %.3f, %.3f) yaw=%.2f deg",
                t_oc[0], t_oc[1], t_oc[2], math.degrees(yaw),
            )

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        if not _finite((p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            return

        # Body pose in camera_init: rotation R_cb and translation t_cb.
        q_cb = (q.x, q.y, q.z, q.w)
        t_cb = (p.x, p.y, p.z)

        self._try_lock_alignment(q_cb, t_cb)
        with self._lock:
            if not self._aligned:
                return
            q_oc = self._q_oc
            t_oc = self._t_oc

        # Body pose in odom: R_ob = R_oc * R_cb, t_ob = t_oc + R_oc * t_cb.
        q_ob = tft.quaternion_multiply(q_oc, q_cb)
        R_ob = _rot_matrix(q_ob)
        t_ob = tuple(np.asarray(t_oc, dtype=np.float64)
                     + R_ob.dot(np.asarray(t_cb, dtype=np.float64)))

        # Lidar pose in odom: q_ol = q_ob * q_bl, t_ol = t_ob + R_ob * t_bl.
        q_ol = tft.quaternion_multiply(q_ob, self.q_bl)
        t_ol = tuple(np.asarray(t_ob, dtype=np.float64)
                     + R_ob.dot(np.asarray(self.t_bl, dtype=np.float64)))

        with self._lock:
            self._br.sendTransform(
                t_ol, q_ol, msg.header.stamp, self.laser_frame, self.map_frame,
            )

        out = Odometry()
        out.header = msg.header
        out.header.frame_id = self.map_frame
        out.child_frame_id = "body"
        out.pose.pose.position.x = t_ob[0]
        out.pose.pose.position.y = t_ob[1]
        out.pose.pose.position.z = t_ob[2]
        out.pose.pose.orientation.x = q_ob[0]
        out.pose.pose.orientation.y = q_ob[1]
        out.pose.pose.orientation.z = q_ob[2]
        out.pose.pose.orientation.w = q_ob[3]
        out.pose.covariance = msg.pose.covariance
        out.twist = msg.twist
        self._odom_pub.publish(out)

    def _scan_cb(self, msg):
        relay = PointCloud()
        relay.header = msg.header
        relay.header.frame_id = self.laser_frame
        relay.points = msg.points
        relay.channels = msg.channels
        self._scan_pub.publish(relay)


def main():
    rospy.init_node("scan_tf_relay")
    ScanTfRelay()
    rospy.spin()


if __name__ == "__main__":
    main()
