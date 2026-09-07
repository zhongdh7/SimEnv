#!/usr/bin/env python3
"""Publish validated odometry as the hazard pipeline's odom-to-base TF edge."""

import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


def _finite(*values):
    return all(math.isfinite(float(value)) for value in values)


def odometry_transform(message, parent_frame="", child_frame=""):
    """Convert one odometry message into a finite, normalized TF transform."""
    parent = parent_frame or message.header.frame_id
    child = child_frame or message.child_frame_id
    if not parent or not child:
        raise ValueError("odometry must provide parent and child frames")

    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    values = (
        position.x,
        position.y,
        position.z,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )
    if not _finite(*values):
        raise ValueError("odometry pose contains NaN or Inf")
    norm = math.sqrt(sum(float(value) ** 2 for value in values[3:]))
    if not math.isfinite(norm) or norm < 1.0e-9:
        raise ValueError("odometry quaternion is invalid")

    transform = TransformStamped()
    transform.header = message.header
    if transform.header.stamp.is_zero():
        transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.x = position.x
    transform.transform.translation.y = position.y
    transform.transform.translation.z = position.z
    transform.transform.rotation.x = orientation.x / norm
    transform.transform.rotation.y = orientation.y / norm
    transform.transform.rotation.z = orientation.z / norm
    transform.transform.rotation.w = orientation.w / norm
    return transform


class HazardOdomTfBridge:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry_fastlio")
        self.parent_frame = rospy.get_param("~parent_frame", "odom")
        self.child_frame = rospy.get_param("~child_frame", "base")
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.subscriber = rospy.Subscriber(
            self.odom_topic, Odometry, self._callback, queue_size=5
        )
        rospy.loginfo(
            "hazard_odom_tf_bridge: %s -> %s from %s",
            self.parent_frame,
            self.child_frame,
            self.odom_topic,
        )

    def _callback(self, message):
        try:
            transform = odometry_transform(
                message, self.parent_frame, self.child_frame
            )
        except (AttributeError, TypeError, ValueError) as error:
            rospy.logwarn_throttle(2.0, "Ignoring invalid odometry for hazard TF: %s", error)
            return
        self.broadcaster.sendTransform(transform)


def main():
    rospy.init_node("hazard_odom_tf_bridge")
    HazardOdomTfBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
