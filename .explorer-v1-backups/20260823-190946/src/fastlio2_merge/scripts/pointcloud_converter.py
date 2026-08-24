#!/usr/bin/env python3
"""Relay SimEnv's legacy ``/scan`` (sensor_msgs/PointCloud) as PointCloud2.

FAST-LIO's MARSIM mode (``lidar_type: 4``) consumes ``sensor_msgs/PointCloud2``
with XYZI fields via ``Preprocess::sim_handler``.  SimEnv's Mid360 Gazebo
plugin publishes the deprecated ``sensor_msgs/PointCloud`` type on ``/scan``,
which FAST-LIO cannot subscribe to directly.  This node relays the same points
as a PointCloud2 on ``/livox/Pointcloud2_local`` without touching the C++
plugin, while ``/scan`` stays available for competition_navigation's 2-D
occupancy-grid builder.
"""

from __future__ import print_function

import math

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud, PointCloud2

FIELDS = [
    pc2.PointField("x", 0, pc2.PointField.FLOAT32, 1),
    pc2.PointField("y", 4, pc2.PointField.FLOAT32, 1),
    pc2.PointField("z", 8, pc2.PointField.FLOAT32, 1),
    pc2.PointField("intensity", 12, pc2.PointField.FLOAT32, 1),
]


class PointCloudConverter(object):
    def __init__(self):
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.cloud_topic = rospy.get_param(
            "~cloud_topic", "/livox/Pointcloud2_local"
        )
        self.min_range = float(rospy.get_param("~min_range", 0.5))
        # SimEnv's Mid360 lidar has a 40 m max range; anything beyond that is a
        # gazebo ray-casting artifact and would blow up FAST-LIO's VoxelGrid
        # bounding box ("Leaf size is too small").
        self.max_range = float(rospy.get_param("~max_range", 40.0))
        self._sub = rospy.Subscriber(
            self.scan_topic, PointCloud, self._callback, queue_size=1,
        )
        self._pub = rospy.Publisher(self.cloud_topic, PointCloud2, queue_size=1)
        self._msg_count = 0
        rospy.loginfo(
            "pointcloud_converter: %s -> %s (PointCloud2, range %.1f..%.1f m)",
            self.scan_topic, self.cloud_topic, self.min_range, self.max_range,
        )

    def _callback(self, msg):
        intensities = None
        for channel in msg.channels:
            if channel.name == "intensity":
                intensities = channel.values
                break

        points = []
        max_mag2 = 0.0
        for i, point in enumerate(msg.points):
            x, y, z = point.x, point.y, point.z
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            r2 = x * x + y * y + z * z
            if r2 > max_mag2:
                max_mag2 = r2
            # SimEnv publishes out-of-range rays as (0,0,0); drop them together
            # with other points inside the lidar blind zone.
            if r2 < self.min_range * self.min_range:
                continue
            # Drop anything beyond the lidar's max range (gazebo ray-casting
            # artifacts) so FAST-LIO's VoxelGrid bounding box stays bounded.
            if r2 > self.max_range * self.max_range:
                continue
            intensity = intensities[i] if intensities is not None else 0.0
            points.append((x, y, z, float(intensity)))

        self._msg_count += 1
        if self._msg_count <= 5:
            rospy.loginfo(
                "pointcloud_converter: scan %d: %d in -> %d kept, max input %.1f m",
                self._msg_count, len(msg.points), len(points), math.sqrt(max_mag2),
            )

        header = msg.header
        cloud = pc2.create_cloud(header, FIELDS, points)
        self._pub.publish(cloud)


def main():
    rospy.init_node("pointcloud_converter")
    PointCloudConverter()
    rospy.spin()


if __name__ == "__main__":
    main()
