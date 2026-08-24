#!/usr/bin/env python3
"""Compensate SimEnv's IMU to FAST-LIO's expected specific force.

SimEnv's trunk IMU publishes ``sensor->LinearAcceleration()``, which is the
body-frame linear acceleration WITHOUT gravity.  FAST-LIO treats
``linear_acceleration`` as the specific force (measured acceleration
including gravity) and normalises by ``mean_acc.norm()``; with gravity
missing that norm is ~0 and the EKF diverges (NaN point coordinates).

This node adds gravity back in the body frame and republishes the result on
a new topic for FAST-LIO to consume.
"""

from __future__ import print_function

import numpy as np
import rospy
import tf.transformations as tft
from sensor_msgs.msg import Imu


class ImuGravityCompensator(object):
    def __init__(self):
        self.imu_topic = rospy.get_param("~imu_topic", "/trunk_imu")
        self.out_topic = rospy.get_param(
            "~out_topic", "/trunk_imu_specific"
        )
        gravity = float(rospy.get_param("~gravity", 9.81))
        # World-frame gravity vector (points down, -z).
        self.g_world = np.array([0.0, 0.0, -gravity])

        self._sub = rospy.Subscriber(
            self.imu_topic, Imu, self._callback, queue_size=20,
        )
        self._pub = rospy.Publisher(self.out_topic, Imu, queue_size=20)
        rospy.loginfo(
            "imu_gravity_compensator: %s -> %s (g=%.2f)",
            self.imu_topic, self.out_topic, gravity,
        )

    def _callback(self, msg):
        # orientation: body -> world. Invert to rotate world gravity into body.
        q = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]
        r_body_to_world = tft.quaternion_matrix(q)[:3, :3]
        g_body = r_body_to_world.T.dot(self.g_world)

        a_body = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])
        specific_force = a_body - g_body

        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration.x = specific_force[0]
        out.linear_acceleration.y = specific_force[1]
        out.linear_acceleration.z = specific_force[2]
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self._pub.publish(out)


def main():
    rospy.init_node("imu_gravity_compensator")
    ImuGravityCompensator()
    rospy.spin()


if __name__ == "__main__":
    main()
