#!/usr/bin/env python3
"""Relay an odometry topic, enforcing strictly-increasing timestamps.

Cartographer's ``OptimizationProblem2D::AddOdometryData`` requires odometry
timestamps to be strictly increasing (``map_by_time.h`` CHECK).  Under Gazebo
``use_sim_time`` the clock can advance in coarse steps, so consecutive odometry
messages occasionally share an identical stamp and crash cartographer_node with
``Check failed: data.time > std::prev(trajectory.end())->first``.

This node subscribes to a raw odometry topic and republishes only messages whose
stamp is strictly newer than the last one published, dropping duplicates and
out-of-order messages.  Point Cartographer's ``odom`` remap at the output topic.
"""
import rospy
from nav_msgs.msg import Odometry

rospy.init_node('odom_dedup')

in_topic = rospy.get_param('~in_topic', '/Odometry_gazebo')
out_topic = rospy.get_param('~out_topic', '/odom_dedup')
# Optional frame renaming.  FAST-LIO2 publishes its odometry with
# child_frame_id "body", but Cartographer's SensorBridge looks up
# tracking_frame <- child_frame_id in TF, and "body" is not part of the
# robot's TF tree (its base frame is "base").  Setting ~child_frame to "base"
# relabels the child frame so Cartographer accepts the odometry directly.
frame_id = rospy.get_param('~frame_id', '')
child_frame = rospy.get_param('~child_frame', '')

pub = rospy.Publisher(out_topic, Odometry, queue_size=10)
last = None
dropped = 0


def cb(msg):
    global last, dropped
    stamp = (msg.header.stamp.secs, msg.header.stamp.nsecs)
    if last is not None and stamp <= last:
        dropped += 1
        if dropped % 100 == 1:
            rospy.logwarn(
                'odom_dedup: dropped %d msg(s) with non-increasing stamp '
                '(last=%d.%09d, got=%d.%09d)'
                % (dropped, last[0], last[1], stamp[0], stamp[1]))
        return
    last = stamp
    if frame_id:
        msg.header.frame_id = frame_id
    if child_frame:
        msg.child_frame_id = child_frame
    pub.publish(msg)


rospy.Subscriber(in_topic, Odometry, cb, queue_size=100)
rospy.loginfo('odom_dedup: %s -> %s (strictly increasing stamps%s%s)',
              in_topic, out_topic,
              ', frame_id=%s' % frame_id if frame_id else '',
              ', child_frame=%s' % child_frame if child_frame else '')
rospy.spin()
