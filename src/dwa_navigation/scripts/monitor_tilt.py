#!/usr/bin/env python3
"""Monitor base tilt from odom→base TF in real time."""
import math
import rospy
import tf

rospy.init_node("monitor_tilt", anonymous=True)
tl = tf.TransformListener()
odom = rospy.get_param("~odom_frame", "odom")
base = rospy.get_param("~base_frame", "base")
rate = rospy.Rate(20)

rospy.loginfo("Monitoring %s → %s tilt...", odom, base)

while not rospy.is_shutdown():
    try:
        tl.waitForTransform(odom, base, rospy.Time(0), rospy.Duration(0.1))
        (_, rot) = tl.lookupTransform(odom, base, rospy.Time(0))
        x, y, z, w = rot
        sinr = 2.0 * (w * x + y * z)
        cosr = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr, cosr)
        sinp = 2.0 * (w * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        tilt = math.acos(math.cos(roll) * math.cos(pitch))
        rospy.loginfo("roll=%.2f pitch=%.2f tilt=%.2f (deg)",
                      math.degrees(roll), math.degrees(pitch), math.degrees(tilt))
    except (tf.Exception, tf.LookupException,
            tf.ConnectivityException, tf.ExtrapolationException):
        pass
    rate.sleep()
