#!/usr/bin/env python3
"""Bridge Cartographer's loop-closure pose into the navigation node's TF.

Cartographer publishes TF ``carto_map -> base`` (loop-closure-corrected), but
``base`` is ALSO the child of the ground-truth ``odom`` frame that
``state_from_gazebo`` broadcasts (``map -> odom -> base``).  That double
parenting makes any tf2 lookup of ``carto_map -> base`` fail with "two or more
unconnected trees", so neither a tf2-based bridge nor the navigation node's own
``lookupTransform(carto_map, laser_livox)`` can resolve it.

This node works around the collision exactly the way ``scan_tf_relay`` does for
FAST-LIO2: it never asks tf2 for the SLAM pose.  Instead it reads the raw
``/tf`` stream, picks out Cartographer's ``carto_map -> base`` transform
directly, and then:

  1. republishes it as ``/carto_odom`` (Odometry, frame ``carto_map``) for the
     navigation node's pose input, and
  2. composes ``carto_map -> base`` with the static ``base -> laser_livox``
     (parsed from ``/tf_static``) and broadcasts ``carto_map -> laser_livox_carto``,
     re-labelling ``/scan`` under that new frame as ``/scan_carto``.

The navigation node is launched with ``map_frame=carto_map``,
``scan_topic=/scan_carto`` and ``odom_topic=/carto_odom``; its single-hop TF
lookup ``carto_map -> laser_livox_carto`` never touches the ambiguous ``base``.
It writes no motion commands and modifies no exploration logic.
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

        rospy.Subscriber("/tf", TFMessage, self._tf_cb, queue_size=50)
        rospy.Subscriber("/tf_static", TFMessage, self._tf_cb, queue_size=10)
        rospy.Subscriber(self.scan_topic, PointCloud, self._scan_cb, queue_size=1)

        self._odom_pub = rospy.Publisher(self.out_odom_topic, Odometry, queue_size=10)
        self._scan_pub = rospy.Publisher(self.out_scan_topic, PointCloud, queue_size=1)

        rospy.loginfo(
            "carto_relay: %s -> %s (odom=%s, scan=%s -> %s)",
            self.carto_map_frame, self.out_laser_frame,
            self.out_odom_topic, self.scan_topic, self.out_scan_topic,
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

    def _publish(self):
        with self._lock:
            pose = self._carto_pose
            ext = self._laser_ext
        if pose is None or ext is None:
            return
        t_cb, q_cb, stamp = pose
        t_bl, q_bl = ext

        # carto_map -> laser_livox_carto: q_cl = q_cb * q_bl,
        #                                  t_cl = t_cb + R_cb @ t_bl.
        q_cl = tft.quaternion_multiply(q_cb, q_bl)
        R_cb = _rot_matrix(q_cb)
        t_cl = tuple(np.asarray(t_cb, dtype=np.float64)
                     + R_cb.dot(np.asarray(t_bl, dtype=np.float64)))

        self._br.sendTransform(
            t_cl, q_cl, stamp, self.out_laser_frame, self.carto_map_frame,
        )

        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = self.carto_map_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = t_cb[0]
        out.pose.pose.position.y = t_cb[1]
        out.pose.pose.position.z = t_cb[2]
        out.pose.pose.orientation.x = q_cb[0]
        out.pose.pose.orientation.y = q_cb[1]
        out.pose.pose.orientation.z = q_cb[2]
        out.pose.pose.orientation.w = q_cb[3]
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
