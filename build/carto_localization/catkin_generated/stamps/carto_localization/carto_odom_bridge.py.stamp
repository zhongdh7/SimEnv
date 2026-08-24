#!/usr/bin/env python3
"""Publish Cartographer's loop-closure-corrected pose as an Odometry message.

Cartographer publishes TF ``carto_map -> base`` (drift-corrected by loop
closure).  The competition navigation node reads the robot pose from an
``nav_msgs/Odometry`` topic rather than TF, and treats that pose as already
expressed in its ``map_frame``.  This node bridges the two: it looks up
``carto_map -> base`` and republishes it as ``nav_msgs/Odometry`` for the nav
node to consume with ``map_frame=carto_map``.

It writes no TF and sends no motion commands -- it only reads TF and publishes
the single odometry topic.
"""
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion

rospy.init_node('carto_odom_bridge')

map_frame = rospy.get_param('~map_frame', 'carto_map')
base_frame = rospy.get_param('~base_frame', 'base')
out_topic = rospy.get_param('~out_topic', '/carto_odom')
rate = float(rospy.get_param('~rate', 20.0))

listener = tf.TransformListener()
pub = rospy.Publisher(out_topic, Odometry, queue_size=10)
r = rospy.Rate(rate)

while not rospy.is_shutdown():
    try:
        trans, rot = listener.lookupTransform(
            map_frame, base_frame, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException,
            tf.ExtrapolationException):
        # Cartographer hasn't published carto_map -> base yet (or TF dropped a
        # frame); skip this cycle and retry.
        r.sleep()
        continue
    msg = Odometry()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = map_frame
    msg.child_frame_id = base_frame
    msg.pose.pose.position = Point(*trans)
    msg.pose.pose.orientation = Quaternion(*rot)
    pub.publish(msg)
    r.sleep()
