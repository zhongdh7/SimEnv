#!/usr/bin/env python3
"""Bridge Cartographer's loop-closure pose into the navigation node's TF.

Cartographer publishes ``carto_map -> base`` (loop-closure-corrected), but that
frame is the ROBOT's own start pose (identity yaw), NOT the building-aligned
``odom`` frame.  The navigation logic (``competition_navigation_node.py``,
copied unchanged) is *not* frame-agnostic: its door/room/corridor/stairs
heuristics assume the map frame is aligned with the building axes (the corridor
runs along +Y and the robot spawns facing +Y).  Fed ``carto_map`` directly, the
whole map appears rotated ~90 deg and the robot drives off sideways instead of
entering the main gate -- exactly the problem ``scan_tf_relay`` solves for
FAST-LIO2.

A further complication is that ``base`` is DOUBLE-parented: ``state_from_gazebo``
broadcasts ``odom -> base`` while Cartographer broadcasts ``carto_map -> base``.
That makes any tf2 lookup of ``carto_map -> base`` fail intermittently ("two or
more unconnected trees"), so neither a tf2-based bridge nor the navigation
node's own ``lookupTransform(carto_map, laser_livox)`` can resolve it.

This node therefore:

  1. reads Cartographer's ``carto_map -> base`` straight off the raw ``/tf``
     stream (bypassing tf2 and the double-parent collision),
  2. re-anchors it into the building-aligned ``odom`` frame using a ONE-TIME
     ground-truth alignment (``/Odometry_gazebo``), exactly like
     ``scan_tf_relay`` does for FAST-LIO2 -- ground truth is used only for this
     single initial anchor, never for continuous localisation -- and
  3. publishes the loop-closure-corrected pose on ``/carto_odom`` (frame
     ``odom``) and the re-labelled scan on ``/scan_carto`` (frame
     ``laser_livox_carto``, parented directly by ``odom``).

The navigation node is launched with ``map_frame=odom``,
``scan_topic=/scan_carto`` and ``odom_topic=/carto_odom``; its behaviour matches
the ground-truth version but with Cartographer's loop-closure correction folded
in.  It writes no motion commands and modifies no exploration logic.
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
from tf2_msgs.msg import TFMessage


def _finite(vals):
    return all(math.isfinite(float(v)) for v in vals)


def _rot_matrix(q):
    return tft.quaternion_matrix(q)[0:3, 0:3]


class CartoRelay(object):
    def __init__(self):
        self.carto_map_frame = rospy.get_param("~carto_map_frame", "carto_map")
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.laser_frame = rospy.get_param("~laser_frame", "laser_livox")
        self.out_laser_frame = rospy.get_param("~out_laser_frame", "laser_livox_carto")
        # Output map frame: the building-aligned odom frame the navigation node
        # expects (its heuristics are NOT frame-agnostic).
        self.map_frame = rospy.get_param("~map_frame", "odom")
        self.gt_odom_topic = rospy.get_param("~gt_odom_topic", "/Odometry_gazebo")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.out_scan_topic = rospy.get_param("~out_scan_topic", "/scan_carto")
        self.out_odom_topic = rospy.get_param("~out_odom_topic", "/carto_odom")

        self._lock = threading.Lock()
        self._br = tf.TransformBroadcaster()

        # carto_map -> base : (t_cb, q_cb, stamp)
        self._carto_pose = None
        # base -> laser_livox : (t_bl, q_bl)  (static extrinsic, may be on /tf
        # or /tf_static depending on the URDF joint type)
        self._laser_ext = None

        # One-time alignment ``odom <- carto_map`` from the first ground-truth
        # pose and Cartographer's first pose.  None until ready.
        self._aligned = False
        self._q_oc = None  # rotation carto_map -> odom
        self._t_oc = None  # translation carto_map -> odom
        self._latest_gt = None  # (t, q) of base in odom, newest /Odometry_gazebo

        rospy.Subscriber("/tf", TFMessage, self._tf_cb, queue_size=50)
        rospy.Subscriber("/tf_static", TFMessage, self._tf_cb, queue_size=10)
        rospy.Subscriber(self.gt_odom_topic, Odometry, self._gt_cb, queue_size=5)
        rospy.Subscriber(self.scan_topic, PointCloud, self._scan_cb, queue_size=1)

        self._odom_pub = rospy.Publisher(self.out_odom_topic, Odometry, queue_size=10)
        self._scan_pub = rospy.Publisher(self.out_scan_topic, PointCloud, queue_size=1)

        rospy.loginfo(
            "carto_relay: %s -> %s -> %s (odom=%s, scan=%s -> %s, gt=%s)",
            self.carto_map_frame, self.base_frame, self.map_frame,
            self.out_odom_topic, self.scan_topic, self.out_scan_topic,
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

    def _tf_cb(self, msg):
        changed = False
        for tr in msg.transforms:
            parent = tr.header.frame_id
            child = tr.child_frame_id
            if parent == self.carto_map_frame and child == self.base_frame:
                p = tr.transform.translation
                q = tr.transform.rotation
                if _finite((p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                    with self._lock:
                        self._carto_pose = (
                            (p.x, p.y, p.z), (q.x, q.y, q.z, q.w), tr.header.stamp,
                        )
                    changed = True
            elif parent == self.base_frame and child == self.laser_frame:
                p = tr.transform.translation
                q = tr.transform.rotation
                if _finite((p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                    with self._lock:
                        if self._laser_ext is None:
                            self._laser_ext = ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
        if changed:
            self._publish()

    def _try_lock_alignment(self, q_cb, t_cb):
        """Derive and lock ``odom <- carto_map`` from the first pose pair."""
        with self._lock:
            if self._aligned:
                return
            gt = self._latest_gt
            if gt is None:
                rospy.logwarn_throttle(
                    5.0,
                    "carto_relay: waiting for %s to anchor the initial pose",
                    self.gt_odom_topic,
                )
                return
            t_ob0, q_ob0 = gt

            # R_oc = R_ob(0) * R_cb(0)^T,  t_oc = t_ob(0) - R_oc * t_cb(0).
            q_oc = tft.quaternion_multiply(q_ob0, tft.quaternion_inverse(q_cb))
            R_oc = _rot_matrix(q_oc)
            t_cb_arr = np.asarray(t_cb, dtype=np.float64)
            t_oc = tuple(np.asarray(t_ob0, dtype=np.float64) - R_oc.dot(t_cb_arr))

            self._q_oc = q_oc
            self._t_oc = t_oc
            self._aligned = True

            yaw = tft.euler_from_quaternion(q_oc)[2]
            rospy.loginfo(
                "carto_relay: alignment locked "
                "t_oc=(%.3f, %.3f, %.3f) yaw=%.2f deg",
                t_oc[0], t_oc[1], t_oc[2], math.degrees(yaw),
            )

    def _publish(self):
        with self._lock:
            pose = self._carto_pose
            ext = self._laser_ext
        if pose is None or ext is None:
            return
        t_cb, q_cb, stamp = pose
        t_bl, q_bl = ext

        self._try_lock_alignment(q_cb, t_cb)
        with self._lock:
            if not self._aligned:
                return
            q_oc = self._q_oc
            t_oc = self._t_oc

        # Loop-closure-corrected body pose in odom:
        #   R_ob = R_oc * R_cb,  t_ob = t_oc + R_oc * t_cb.
        q_ob = tft.quaternion_multiply(q_oc, q_cb)
        R_oc = _rot_matrix(q_oc)
        R_ob = _rot_matrix(q_ob)
        t_ob = tuple(np.asarray(t_oc, dtype=np.float64)
                     + R_oc.dot(np.asarray(t_cb, dtype=np.float64)))

        # Lidar pose in odom: q_ol = q_ob * q_bl, t_ol = t_ob + R_ob * t_bl.
        q_ol = tft.quaternion_multiply(q_ob, q_bl)
        t_ol = tuple(np.asarray(t_ob, dtype=np.float64)
                     + R_ob.dot(np.asarray(t_bl, dtype=np.float64)))

        self._br.sendTransform(
            t_ol, q_ol, stamp, self.out_laser_frame, self.map_frame,
        )

        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = t_ob[0]
        out.pose.pose.position.y = t_ob[1]
        out.pose.pose.position.z = t_ob[2]
        out.pose.pose.orientation.x = q_ob[0]
        out.pose.pose.orientation.y = q_ob[1]
        out.pose.pose.orientation.z = q_ob[2]
        out.pose.pose.orientation.w = q_ob[3]
        self._odom_pub.publish(out)

    def _scan_cb(self, msg):
        relay = PointCloud()
        relay.header = msg.header
        relay.header.frame_id = self.out_laser_frame
        relay.points = msg.points
        relay.channels = msg.channels
        self._scan_pub.publish(relay)


def main():
    rospy.init_node("carto_relay")
    CartoRelay()
    rospy.spin()


if __name__ == "__main__":
    main()
