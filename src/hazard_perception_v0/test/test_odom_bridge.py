#!/usr/bin/env python3

import math
import os
import sys
import unittest

import rospy
from nav_msgs.msg import Odometry


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "scripts"))

from hazard_odom_tf_bridge import normalize_odometry_message


class HazardOdomBridgeTest(unittest.TestCase):
    def test_conversion_preserves_stamp_pose_twist_and_covariance(self):
        message = Odometry()
        message.header.stamp = rospy.Time(12, 345)
        message.header.frame_id = "camera_init"
        message.child_frame_id = "body"
        message.pose.pose.position.x = 1.0
        message.pose.pose.position.y = -2.0
        message.pose.pose.position.z = 0.3
        message.pose.pose.orientation.z = 2.0
        message.pose.pose.orientation.w = 2.0
        message.pose.covariance[0] = 0.25
        message.twist.twist.linear.x = 0.4
        message.twist.covariance[0] = 0.5

        result = normalize_odometry_message(message, "odom", "base")

        self.assertEqual(result.header.stamp, message.header.stamp)
        self.assertEqual(result.header.frame_id, "odom")
        self.assertEqual(result.child_frame_id, "base")
        self.assertEqual(result.pose.pose.position.x, 1.0)
        self.assertEqual(result.pose.covariance[0], 0.25)
        self.assertEqual(result.twist.twist.linear.x, 0.4)
        self.assertEqual(result.twist.covariance[0], 0.5)
        self.assertAlmostEqual(result.pose.pose.orientation.z, 1.0 / math.sqrt(2.0))
        self.assertAlmostEqual(result.pose.pose.orientation.w, 1.0 / math.sqrt(2.0))
        self.assertEqual(message.header.frame_id, "camera_init")

    def test_conversion_rejects_nonfinite_position(self):
        message = Odometry()
        message.header.frame_id = "odom"
        message.child_frame_id = "base"
        message.pose.pose.position.x = float("nan")
        message.pose.pose.orientation.w = 1.0
        with self.assertRaises(ValueError):
            normalize_odometry_message(message)

    def test_conversion_rejects_nonfinite_twist(self):
        message = Odometry()
        message.header.frame_id = "odom"
        message.child_frame_id = "base"
        message.pose.pose.orientation.w = 1.0
        message.twist.twist.linear.x = float("inf")
        with self.assertRaises(ValueError):
            normalize_odometry_message(message)


if __name__ == "__main__":
    unittest.main()
