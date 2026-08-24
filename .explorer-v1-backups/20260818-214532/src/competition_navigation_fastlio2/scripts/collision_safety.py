#!/usr/bin/env python3
"""Collision-safety layer (independent node -- does NOT modify the exploration logic).

The navigation controller is a pure waypoint tracker with no local obstacle
avoidance: it computes ``linear.x = linear_gain * body_x`` toward the next
waypoint and publishes it directly on ``/cmd_vel``.  When the planner routes a
path through a wall (or the robot overshoots a waypoint into one), the
controller drives straight into it -- the wall-hugging / fall scenario.

This node sits between the navigation node and the robot's servo so the robot
never pushes into a mapped obstacle:

    competition_navigation_node  --/cmd_vel_raw-->  collision_safety  --/cmd_vel-->  unitree_gazebo_servo

It reads the navigation node's own published 2-D occupancy map (already
flattened and self-filtered by the nav), plus the robot pose, and zeroes the
forward/lateral velocity when an occupied cell lies inside a small safety box
ahead of the direction of travel.  The yaw-rate command is left intact so the
robot can still turn away.  A stale command (nav died) also drops to zero.

Tunables (all via private params, defaults chosen for the A1 in SimEnv):
  safety_dist        -- stop when an obstacle is within this many metres ahead
  safety_half_width  -- half-width of the safety box (kept < a doorway's half
                        width so passing through doors does not false-trigger)
  min_check_dist     -- ignore cells closer than this (the robot's own body)
"""

import math

import rospy
import tf.transformations as tft
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid

OCCUPIED = 100


class CollisionSafety(object):
    def __init__(self):
        self.cmd_in = rospy.get_param("~cmd_in", "/cmd_vel_raw")
        self.cmd_out = rospy.get_param("~cmd_out", "/cmd_vel")
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry_fastlio")
        self.map_topic = rospy.get_param("~map_topic", "/competition_navigation/map")
        self.rate = float(rospy.get_param("~rate", 20.0))
        self.stale_timeout = rospy.Duration(
            float(rospy.get_param("~stale_timeout", 0.6))
        )

        self.safety_dist = float(rospy.get_param("~safety_dist", 0.6))
        self.safety_half_width = float(rospy.get_param("~safety_half_width", 0.35))
        self.min_check_dist = float(rospy.get_param("~min_check_dist", 0.25))

        self.map = None
        self.pose = None  # (x, y, z, yaw) in the map frame
        self.last_cmd = Twist()
        self.last_cmd_stamp = None

        self.pub = rospy.Publisher(self.cmd_out, Twist, queue_size=5)
        rospy.Subscriber(self.cmd_in, Twist, self._cmd_cb)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb)
        rospy.Subscriber(self.map_topic, OccupancyGrid, self._map_cb)
        rospy.loginfo(
            "collision_safety: %s -> %s (dist=%.2fm half-width=%.2fm rate=%.0fHz)",
            self.cmd_in, self.cmd_out,
            self.safety_dist, self.safety_half_width, self.rate,
        )

    def _map_cb(self, msg):
        self.map = msg

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose = (p.x, p.y, p.z, yaw)

    def _cmd_cb(self, msg):
        self.last_cmd = msg
        self.last_cmd_stamp = rospy.Time.now()

    def _occ(self, x, y):
        m = self.map
        if m is None:
            return None
        res = m.info.resolution
        cx = int(math.floor((x - m.info.origin.position.x) / res))
        cy = int(math.floor((y - m.info.origin.position.y) / res))
        if 0 <= cx < m.info.width and 0 <= cy < m.info.height:
            return m.data[cy * m.info.width + cx]
        return None

    def _blocked(self, sign):
        """True if an occupied cell sits in the safety box ahead of travel.

        sign = +1 for forward motion, -1 for reverse.  Samples a short grid of
        map cells along the heading axis and the two edges of the box."""
        x, y, _, yaw = self.pose
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        d = self.min_check_dist
        while d <= self.safety_dist + 1e-6:
            for lat in (-self.safety_half_width, 0.0, self.safety_half_width):
                px = x + sign * d * cos_yaw + lat * -sin_yaw
                py = y + sign * d * sin_yaw + lat * cos_yaw
                if self._occ(px, py) == OCCUPIED:
                    return True
            d += 0.2
        return False

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            cmd = self.last_cmd
            now = rospy.Time.now()
            if self.last_cmd_stamp is not None and now - self.last_cmd_stamp > self.stale_timeout:
                cmd = Twist()  # nav died or stalled -> hold still
                rospy.logwarn_throttle(2.0, "collision_safety: cmd_vel stale, holding zero")
            elif self.pose is not None and self.map is not None:
                lin_x = cmd.linear.x
                if lin_x > 0.01 and self._blocked(+1):
                    cmd.linear.x = 0.0
                    cmd.linear.y = 0.0
                    rospy.logwarn_throttle(
                        1.0, "collision_safety: forward blocked, zeroing velocity"
                    )
                elif lin_x < -0.01 and self._blocked(-1):
                    cmd.linear.x = 0.0
                    cmd.linear.y = 0.0
                    rospy.logwarn_throttle(
                        1.0, "collision_safety: reverse blocked, zeroing velocity"
                    )
            self.pub.publish(cmd)
            r.sleep()


def main():
    rospy.init_node("collision_safety")
    CollisionSafety().spin()


if __name__ == "__main__":
    main()
