#!/usr/bin/env python3
"""Adapt any standard nav_msgs/Odometry source for hazard perception.

The adapter is deliberately LIO-package agnostic.  It never imports FAST-LIO2,
ws_livox, Gazebo odometry or simulator truth.  Disable ``~publish_tf`` when
another node already owns the same parent->base transform.
"""

import copy
import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from hazard_perception_v0.pose_interfaces import normalize_odometry_data


def normalize_odometry_message(message, parent_override="auto", base_override="base"):
    """Return a validated, normalized Odometry copy (testable without a node)."""
    pose = message.pose.pose
    normalized = normalize_odometry_data(
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
        message.header.frame_id,
        message.child_frame_id,
        parent_override,
        base_override,
    )
    auxiliary_values = list(message.pose.covariance) + list(message.twist.covariance) + [
        message.twist.twist.linear.x,
        message.twist.twist.linear.y,
        message.twist.twist.linear.z,
        message.twist.twist.angular.x,
        message.twist.twist.angular.y,
        message.twist.twist.angular.z,
    ]
    if not all(math.isfinite(float(value)) for value in auxiliary_values):
        raise ValueError("odometry covariance/twist contains NaN/Inf")
    result = copy.deepcopy(message)
    result.header.frame_id = normalized["parent_frame"]
    result.child_frame_id = normalized["child_frame"]
    quaternion = normalized["quaternion_xyzw"]
    result.pose.pose.orientation.x = quaternion[0]
    result.pose.pose.orientation.y = quaternion[1]
    result.pose.pose.orientation.z = quaternion[2]
    result.pose.pose.orientation.w = quaternion[3]
    return result


class HazardOdomTfBridge(object):
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.parent_frame = rospy.get_param("~parent_frame", "auto")
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.publish_tf = bool(rospy.get_param("~publish_tf", True))
        self.output_topic = rospy.get_param("~output_odom_topic", "/hazard_pose/odom")
        self.publisher = (
            rospy.Publisher(self.output_topic, Odometry, queue_size=10)
            if str(self.output_topic).strip() else None
        )
        self.broadcaster = tf2_ros.TransformBroadcaster() if self.publish_tf else None
        self.subscriber = rospy.Subscriber(
            self.odom_topic, Odometry, self._callback, queue_size=20
        )
        rospy.loginfo(
            "hazard odometry adapter: input=%s output=%s parent=%s base=%s publish_tf=%s",
            self.odom_topic, self.output_topic, self.parent_frame, self.base_frame,
            self.publish_tf,
        )

    def _callback(self, message):
        try:
            normalized = normalize_odometry_message(
                message, self.parent_frame, self.base_frame
            )
        except (TypeError, ValueError) as error:
            rospy.logwarn_throttle(2.0, "Dropping invalid hazard odometry: %s", error)
            return
        if self.publisher:
            self.publisher.publish(normalized)
        if not self.broadcaster:
            return
        transform = TransformStamped()
        transform.header.stamp = normalized.header.stamp
        transform.header.frame_id = normalized.header.frame_id
        transform.child_frame_id = normalized.child_frame_id
        transform.transform.translation.x = normalized.pose.pose.position.x
        transform.transform.translation.y = normalized.pose.pose.position.y
        transform.transform.translation.z = normalized.pose.pose.position.z
        transform.transform.rotation = normalized.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main():
    rospy.init_node("hazard_odom_tf_bridge")
    HazardOdomTfBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
