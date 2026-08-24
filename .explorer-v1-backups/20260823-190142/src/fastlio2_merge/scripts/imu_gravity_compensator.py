#!/usr/bin/env python3
"""Relay SimEnv's trunk IMU to FAST-LIO2's expected topic (gravity already present).

SimEnv's trunk IMU plugin ``libsimenv_gazebo_ros_imu_sensor.so`` publishes
``sensor->LinearAcceleration()``, which is the SPECIFIC FORCE: the accelerometer
reading that already INCLUDES gravity.  For a stationary upright robot that is
~+9.8 m/s^2 along body +z, which is exactly what FAST-LIO2 expects
(``linear_acceleration`` is specific force per REP-145, and FAST-LIO2 normalises
by ``mean_acc.norm()`` during initialisation).

An earlier version of this node ADDED gravity back on the assumption that the
IMU was gravity-free, but with the plugin now publishing specific force that
double-counted gravity (~19.6 m/s^2), which biased FAST-LIO2's gravity estimate
and made its translation estimate drift metre-scale from the first motion.

This node now relays the acceleration unchanged so FAST-LIO2 receives the true
specific force, and only keeps the topic indirection (/trunk_imu ->
/trunk_imu_specific) so the config does not need to change.
"""

from __future__ import print_function

import rospy
from sensor_msgs.msg import Imu


class ImuGravityCompensator(object):
    def __init__(self):
        self.imu_topic = rospy.get_param("~imu_topic", "/trunk_imu")
        self.out_topic = rospy.get_param(
            "~out_topic", "/trunk_imu_specific"
        )
        self._sub = rospy.Subscriber(
            self.imu_topic, Imu, self._callback, queue_size=20,
        )
        self._pub = rospy.Publisher(self.out_topic, Imu, queue_size=20)
        rospy.loginfo(
            "imu_gravity_compensator: relaying %s -> %s (specific force, "
            "gravity already present)",
            self.imu_topic, self.out_topic,
        )

    def _callback(self, msg):
        # The plugin already publishes specific force (gravity included); relay
        # it verbatim so FAST-LIO2's gravity initialisation sees ~9.8 m/s^2.
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration
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
