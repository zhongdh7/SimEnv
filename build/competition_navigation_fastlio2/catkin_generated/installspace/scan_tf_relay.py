#!/usr/bin/env python3
"""Bridge FAST-LIO2 odometry into the navigation node's TF expectations.

The navigation node (``competition_navigation_node.py``, copied unchanged) reads
its robot pose from an ``nav_msgs/Odometry`` message and transforms every scan
point via ``lookupTransform(map_frame, scan_frame)``.  With Gazebo ground truth
that works because ``state_from_gazebo`` already broadcasts ``odom -> base`` and
``robot_state_publisher`` broadcasts ``base -> laser_livox``.

FAST-LIO2 instead publishes:

  * ``/Odometry`` in the ``camera_init`` frame with child ``body``, and
  * TF ``camera_init -> body`` only.

``body`` is the same physical link as the robot's ``base`` (the IMU/trunk), but
the ground-truth TF tree already parents ``base`` to ``odom``.  Re-parenting
``base`` to ``camera_init`` would give ``base`` two TF parents and break the
tree, so this node does NOT touch ``base``/``laser_livox``.  Instead it:

  1. broadcasts a NEW dynamic transform ``camera_init -> laser_livox_fastlio``
     computed from FAST-LIO2's pose plus the known lidar extrinsic, and
  2. re-publishes ``/scan`` under the re-labelled frame ``laser_livox_fastlio``
     (points unchanged).

The navigation node is then launched with ``map_frame=camera_init``,
``scan_topic=/scan_fastlio`` and ``odom_topic=/Odometry``; its TF lookups and
sensor-origin logic behave exactly as the ground-truth version.
"""

from __future__ import print_function

import math
import threading

import rospy
import tf
import tf.transformations as tft
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud


def _finite(vals):
    return all(math.isfinite(float(v)) for v in vals)


class ScanTfRelay(object):
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.out_scan_topic = rospy.get_param("~out_scan_topic", "/scan_fastlio")
        self.map_frame = rospy.get_param("~map_frame", "camera_init")
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

        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=5)
        rospy.Subscriber(self.scan_topic, PointCloud, self._scan_cb, queue_size=1)
        self._scan_pub = rospy.Publisher(
            self.out_scan_topic, PointCloud, queue_size=1,
        )

        rospy.loginfo(
            "scan_tf_relay: %s -> %s (odom=%s, laser=%s, t_bl=%s)",
            self.scan_topic, self.out_scan_topic,
            self.odom_topic, self.laser_frame, self.t_bl,
        )

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        if not _finite((p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            return

        # Body pose in world: rotation R_wb and translation t_wb.
        q_wb = (q.x, q.y, q.z, q.w)
        R_wb = tft.quaternion_matrix(q_wb)[0:3, 0:3]
        t_wb = (p.x, p.y, p.z)

        # Lidar pose in world: q_wl = q_wb * q_bl, t_wl = t_wb + R_wb * t_bl.
        q_wl = tft.quaternion_multiply(q_wb, self.q_bl)
        t_wl = (
            t_wb[0] + R_wb[0, 0] * self.t_bl[0] + R_wb[0, 1] * self.t_bl[1] + R_wb[0, 2] * self.t_bl[2],
            t_wb[1] + R_wb[1, 0] * self.t_bl[0] + R_wb[1, 1] * self.t_bl[1] + R_wb[1, 2] * self.t_bl[2],
            t_wb[2] + R_wb[2, 0] * self.t_bl[0] + R_wb[2, 1] * self.t_bl[1] + R_wb[2, 2] * self.t_bl[2],
        )

        with self._lock:
            self._br.sendTransform(
                t_wl, q_wl, msg.header.stamp, self.laser_frame, self.map_frame,
            )

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
