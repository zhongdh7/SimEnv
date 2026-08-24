#!/usr/bin/env python3
"""Convert SimEnv's legacy 3-D /scan to a 2-D LaserScan for Cartographer.

SimEnv's Gazebo plugin publishes a *3-D* Livox Mid-360 point cloud on /scan
using the deprecated ``sensor_msgs/PointCloud`` type (frame ``laser_livox``,
pitched +45 deg about Y: ``laser_livox_joint rpy="0 0.785 0" xyz="0.2 0 0.08"``).
Cartographer 2-D consumes ``sensor_msgs/LaserScan``, not the legacy PointCloud,
and cannot use a 3-D cloud directly in 2-D mode.

This node:
  1. subscribes to /scan (PointCloud),
  2. transforms every point laser_livox -> base using the *static* TF broadcast
     by robot_state_publisher (cached after the first lookup),
  3. discards points outside a vertical band around the lidar height so ground
     and ceiling returns do not turn into a ring of phantom obstacles,
  4. projects each surviving point onto the horizontal plane (x, y), buckets it
     by bearing and keeps the closest range per bucket, and
  5. publishes the result as /scan_2d (LaserScan, frame ``base``).

Publishing in frame ``base`` == Cartographer's ``tracking_frame`` means
Cartographer needs no extra TF lookup for the scan (identity transform).
"""

from __future__ import print_function

import math

import numpy as np
import rospy
import tf
import tf.transformations as tft
from sensor_msgs.msg import LaserScan, PointCloud


class PointCloudToLaserScan(object):
    def __init__(self):
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.laser_topic = rospy.get_param("~laser_topic", "/scan_2d")
        self.target_frame = rospy.get_param("~target_frame", "base")
        self.source_frame = rospy.get_param("~source_frame", "laser_livox")

        # Angular resolution: number of buckets across the full 2*pi sweep.
        # 720 -> 0.5 deg.
        self.angle_bins = int(rospy.get_param("~angle_bins", 720))
        self.range_min = float(rospy.get_param("~range_min", 0.3))
        self.range_max = float(rospy.get_param("~range_max", 30.0))
        # Vertical band (in the target/base frame) kept after the TF transform.
        # The A1 trunk sits ~0.35 m above the floor and the lidar a further
        # 0.08 m, so ground returns land near z ~= -0.35 m and are rejected by
        # min_z; high ceiling returns are rejected by max_z.
        self.min_z = float(rospy.get_param("~min_z", -0.15))
        self.max_z = float(rospy.get_param("~max_z", 1.0))

        self._tf = tf.TransformListener()
        self._tf_timeout = rospy.Duration(1.0)
        self._cached = None  # (translation, quaternion) of target <- source

        self._pub = rospy.Publisher(self.laser_topic, LaserScan, queue_size=1)
        rospy.Subscriber(self.scan_topic, PointCloud, self._callback, queue_size=1)

        rospy.loginfo(
            "pointcloud_to_laserscan: %s -> %s (frame %s, %d bins, "
            "z=[%.2f, %.2f], range=[%.1f, %.1f])",
            self.scan_topic, self.laser_topic, self.target_frame,
            self.angle_bins, self.min_z, self.max_z,
            self.range_min, self.range_max,
        )

    def _transform(self):
        """Return the cached static transform (t, q) of target <- source."""
        if self._cached is not None:
            return self._cached
        try:
            self._tf.waitForTransform(
                self.target_frame, self.source_frame, rospy.Time(0),
                self._tf_timeout,
            )
            (t, q) = self._tf.lookupTransform(
                self.target_frame, self.source_frame, rospy.Time(0),
            )
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as exc:
            rospy.logwarn_throttle(
                5.0, "TF %s <- %s unavailable: %s",
                self.target_frame, self.source_frame, exc,
            )
            return None
        self._cached = (t, q)
        rpy = tft.euler_from_quaternion(q)
        rospy.loginfo(
            "pointcloud_to_laserscan: cached %s <- %s "
            "t=(%.3f, %.3f, %.3f) rpy=(%.1f, %.1f, %.1f) deg",
            self.target_frame, self.source_frame,
            t[0], t[1], t[2],
            math.degrees(rpy[0]), math.degrees(rpy[1]), math.degrees(rpy[2]),
        )
        return self._cached

    def _callback(self, msg):
        if not msg.points:
            return
        transform = self._transform()
        if transform is None:
            return
        t, q = transform
        rmat = tft.quaternion_matrix(q)[0:3, 0:3].astype(np.float64)
        tvec = np.asarray(t, dtype=np.float64)

        # Pack points into an (n, 3) array and transform to the target frame.
        xyz = np.array([[p.x, p.y, p.z] for p in msg.points], dtype=np.float64)
        t_xyz = xyz.dot(rmat.T) + tvec

        px = t_xyz[:, 0]
        py = t_xyz[:, 1]
        pz = t_xyz[:, 2]
        rng = np.hypot(px, py)
        ang = np.arctan2(py, px)

        keep = (
            (pz >= self.min_z) & (pz <= self.max_z) &
            (rng >= self.range_min) & (rng <= self.range_max) &
            np.isfinite(rng)
        )
        ang = ang[keep]
        rng = rng[keep]

        angle_increment = 2.0 * math.pi / self.angle_bins
        bins = ((ang + math.pi) / angle_increment).astype(np.int64)
        np.clip(bins, 0, self.angle_bins - 1, out=bins)

        # Closest range per angular bucket; inf = no return (missing data).
        ranges = np.full(self.angle_bins, float("inf"))
        np.minimum.at(ranges, bins, rng)

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.target_frame
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = [float(v) for v in ranges]
        scan.intensities = []
        self._pub.publish(scan)


def main():
    rospy.init_node("pointcloud_to_laserscan")
    PointCloudToLaserScan()
    rospy.spin()


if __name__ == "__main__":
    main()
