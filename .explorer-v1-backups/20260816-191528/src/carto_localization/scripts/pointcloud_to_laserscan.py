#!/usr/bin/env python3
"""Convert SimEnv's legacy /scan (sensor_msgs/PointCloud) to a flat 2D LaserScan.

The SimEnv Livox Mid-360 plugin publishes the deprecated sensor_msgs/PointCloud
type (3-D, non-repetitive, and pitched 45 deg about Y).  Cartographer 2D needs a
flat sensor_msgs/LaserScan lying in a horizontal plane.  This node:

  1. subscribes to the legacy PointCloud,
  2. transforms each point from the cloud's frame (``laser_livox``) into a
     horizontal target frame (default ``base``) using the static transform that
     the sim's robot_state_publisher broadcasts,
  3. projects the transformed points onto the target frame's XY plane and keeps
     the minimum range per angular bin,
  4. publishes a sensor_msgs/LaserScan in the target frame.

Empty angular bins are reported as ``inf`` ("no return"), matching ROS
convention and Cartographer's expectation.

Note: the static transform is cached, so the node assumes ``target_frame`` and
the cloud's frame are rigidly connected (true for base <-> laser_livox).
"""

from __future__ import print_function

import math

import numpy as np
import rospy
import tf
import tf.transformations as tft
from sensor_msgs.msg import LaserScan, PointCloud

INF = float("inf")


class PointCloudToLaserScan(object):
    def __init__(self):
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.laser_topic = rospy.get_param("~laser_topic", "/scan_2d")
        self.target_frame = rospy.get_param("~target_frame", "base")
        self.angle_bins = int(rospy.get_param("~angle_bins", 720))
        self.range_min = float(rospy.get_param("~range_min", 0.1))
        self.range_max = float(rospy.get_param("~range_max", 40.0))

        self._tf_listener = tf.TransformListener(cache_time=rospy.Duration(10.0))
        self._matrix = None       # cached 4x4: target <- source
        self._source_frame = None

        self._pub = rospy.Publisher(self.laser_topic, LaserScan, queue_size=1)
        self._sub = rospy.Subscriber(
            self.scan_topic, PointCloud, self._callback, queue_size=1,
        )
        rospy.loginfo(
            "pointcloud_to_laserscan: %s -> %s (LaserScan, frame %s, %d bins)",
            self.scan_topic, self.laser_topic, self.target_frame, self.angle_bins,
        )

    def _resolve_transform(self, source_frame):
        if source_frame == self.target_frame:
            self._matrix = np.eye(4)
            return True
        try:
            translation, rotation = self._tf_listener.lookupTransform(
                self.target_frame, source_frame, rospy.Time(0)
            )
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            rospy.logwarn_throttle(
                5.0, "TF unavailable: %s -> %s", self.target_frame, source_frame
            )
            return False
        matrix = tft.quaternion_matrix(rotation)
        matrix[0:3, 3] = translation
        self._matrix = matrix
        return True

    def _callback(self, msg):
        source_frame = msg.header.frame_id
        if not source_frame:
            return
        if self._source_frame != source_frame or self._matrix is None:
            self._source_frame = source_frame
            if not self._resolve_transform(source_frame):
                return

        if not msg.points:
            return

        # (N, 4) homogeneous points in the source frame.
        pts = np.empty((len(msg.points), 4), dtype=np.float64)
        pts[:, 0] = [p.x for p in msg.points]
        pts[:, 1] = [p.y for p in msg.points]
        pts[:, 2] = [p.z for p in msg.points]
        pts[:, 3] = 1.0

        # Transform to the target frame, then project onto its XY plane.
        world = pts.dot(self._matrix.T)
        x = world[:, 0]
        y = world[:, 1]
        rng = np.hypot(x, y)
        ang = np.arctan2(y, x)

        valid = (
            np.isfinite(rng)
            & (rng > self.range_min)
            & (rng <= self.range_max)
            & ((np.abs(x) > 1e-9) | (np.abs(y) > 1e-9))
        )
        rng = rng[valid]
        ang = ang[valid]
        if rng.size == 0:
            return

        angle_min = -math.pi
        angle_increment = 2.0 * math.pi / self.angle_bins

        idx = np.floor((ang - angle_min) / angle_increment).astype(np.int64)
        idx = np.clip(idx, 0, self.angle_bins - 1)

        # Minimum range per angular bin (empty bins stay inf).
        ranges = np.full(self.angle_bins, INF, dtype=np.float32)
        np.minimum.at(ranges, idx, rng.astype(np.float32))

        scan = LaserScan()
        scan.header = msg.header
        scan.header.frame_id = self.target_frame
        scan.angle_min = angle_min
        scan.angle_max = math.pi
        scan.angle_increment = angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.tolist()
        scan.intensities = []
        self._pub.publish(scan)


def main():
    rospy.init_node("pointcloud_to_laserscan")
    PointCloudToLaserScan()
    rospy.spin()


if __name__ == "__main__":
    main()
