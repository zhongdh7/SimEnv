#!/usr/bin/env python3
"""Relay FAST-LIO odometry and point cloud from ``camera_init`` to ``odom``.

FAST-LIO hardcodes ``camera_init`` as its output frame_id.  This node
re-publishes the odometry and accumulated point cloud with a configurable
target frame (default ``odom``) so downstream nodes (competition_navigation)
can consume a standard odometry frame without modifying the FAST-LIO C++
source.
"""

from __future__ import print_function

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


class FrameRepublisher(object):
    def __init__(self):
        self.source_frame = str(
            rospy.get_param("~source_frame", "camera_init")
        )
        self.target_frame = str(
            rospy.get_param("~target_frame", "odom")
        )
        self.child_frame = str(
            rospy.get_param("~child_frame", "body")
        )

        rospy.Subscriber(
            "/Odometry", Odometry, self._odom_callback, queue_size=5,
        )
        self._odom_pub = rospy.Publisher(
            "/tf_odom", Odometry, queue_size=5,
        )

        rospy.Subscriber(
            "/cloud_registered", PointCloud2, self._cloud_callback, queue_size=2,
        )
        self._cloud_pub = rospy.Publisher(
            "/cloud_registered_odom", PointCloud2, queue_size=2,
        )

        rospy.loginfo(
            "frame_republisher: %s -> %s (child=%s)",
            self.source_frame, self.target_frame, self.child_frame,
        )

    def _odom_callback(self, msg):
        if msg.header.frame_id.strip("/") != self.source_frame:
            rospy.logwarn_throttle(
                10.0,
                "expected odometry frame '%s', got '%s'",
                self.source_frame, msg.header.frame_id,
            )

        relay = Odometry()
        relay.header = msg.header
        relay.header.frame_id = self.target_frame
        relay.child_frame_id = self.child_frame
        relay.pose = msg.pose
        relay.twist = msg.twist
        self._odom_pub.publish(relay)

    def _cloud_callback(self, msg):
        relay = PointCloud2()
        relay.header = msg.header
        relay.header.frame_id = self.target_frame
        relay.height = msg.height
        relay.width = msg.width
        relay.fields = msg.fields
        relay.is_bigendian = msg.is_bigendian
        relay.point_step = msg.point_step
        relay.row_step = msg.row_step
        relay.data = msg.data
        relay.is_dense = msg.is_dense
        self._cloud_pub.publish(relay)


def main():
    rospy.init_node("frame_republisher")
    FrameRepublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
