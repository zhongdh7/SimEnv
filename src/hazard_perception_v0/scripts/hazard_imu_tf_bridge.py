#!/usr/bin/env python3
"""Publish startup-relative IMU orientation for fixed-platform development.

This adapter never integrates acceleration and never claims valid XYZ, odometry
or global position.  It is unsuitable for formal global hazard localization.
"""

import json

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from hazard_perception_v0.pose_interfaces import (
    normalize_quaternion,
    orientation_only_status,
    relative_orientation,
)


class HazardImuTfBridge(object):
    def __init__(self):
        self.imu_topic = rospy.get_param("~imu_topic", "/trunk_imu")
        self.reference_frame = rospy.get_param(
            "~reference_frame", "hazard_imu_reference"
        )
        self.orientation_frame = rospy.get_param(
            "~orientation_frame", "hazard_imu_orientation"
        )
        self.status_topic = rospy.get_param(
            "~status_topic", "/hazard_pose/imu_status"
        )
        self.initial_orientation = None
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.status_publisher = rospy.Publisher(
            self.status_topic, String, queue_size=1, latch=True
        )
        self.subscriber = rospy.Subscriber(
            self.imu_topic, Imu, self._callback, queue_size=50
        )
        rospy.logwarn(
            "IMU mode is orientation-only and does not provide globally valid "
            "translated hazard coordinates. input=%s frames=%s->%s",
            self.imu_topic, self.reference_frame, self.orientation_frame,
        )

    def _publish_status(self, valid, detail=""):
        payload = orientation_only_status(valid, detail)
        self.status_publisher.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _callback(self, message):
        try:
            if message.orientation_covariance[0] < 0.0:
                raise ValueError("IMU message declares orientation unavailable")
            current = normalize_quaternion((
                message.orientation.x, message.orientation.y,
                message.orientation.z, message.orientation.w,
            ))
            if self.initial_orientation is None:
                self.initial_orientation = current
                rospy.loginfo("Captured initial IMU orientation q0")
            relative = relative_orientation(self.initial_orientation, current)
        except (TypeError, ValueError) as error:
            self._publish_status(False, str(error))
            rospy.logwarn_throttle(2.0, "Dropping invalid hazard IMU orientation: %s", error)
            return

        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self.reference_frame
        transform.child_frame_id = self.orientation_frame
        # Zero is an explicit absent translation, not an estimated position.
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = relative[0]
        transform.transform.rotation.y = relative[1]
        transform.transform.rotation.z = relative[2]
        transform.transform.rotation.w = relative[3]
        self.broadcaster.sendTransform(transform)
        self._publish_status(True)


def main():
    rospy.init_node("hazard_imu_tf_bridge")
    HazardImuTfBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
