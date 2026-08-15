#!/usr/bin/env python3
"""Save the FAST-LIO accumulated point cloud to PCD after exploration finishes.

Subscribes to ``/cloud_registered`` and ``/competition_navigation/status``.

FAST-LIO's ``/cloud_registered`` only carries the CURRENT frame (see
``laserMapping.cpp::publish_frame_world``), not the accumulated map, so this
node merges every frame into one growing point cloud (voxel-grid deduplicated)
and writes that merged map to PCD when the status reports
``returned_home full_map=True``, or on Ctrl-C / ROS shutdown.

Points arrive in the ``camera_init`` (FAST-LIO world) frame, so they are
accumulated as-is -- no transform is required.  The dedup voxel size defaults
to 0.5 m to match FAST-LIO's ``filter_size_map``, keeping the merged cloud
bounded in memory while reproducing the map at the same resolution.
"""

from __future__ import print_function

import math
import os

import rospkg
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

COMPLETION_MARKER = "returned_home full_map=True"


class PcdSaver(object):
    def __init__(self):
        self.cloud_topic = rospy.get_param("~cloud_topic", "/cloud_registered")
        self.status_topic = rospy.get_param(
            "~status_topic", "/competition_navigation/status"
        )
        self.pcd_dir = rospy.get_param("~pcd_dir", "")
        if not self.pcd_dir:
            self.pcd_dir = os.path.join(
                rospkg.RosPack().get_path("fastlio_slam_merge"), "PCD"
            )
        # Voxel size (m) used to deduplicate the merged cloud.  FAST-LIO stores
        # its own map at filter_size_map = 0.5 m, so this reproduces the map at
        # the same resolution while keeping memory bounded.
        self.voxel_size = float(rospy.get_param("~voxel_size", 0.5))

        # Merged map: voxel key (ix, iy, iz) -> (x, y, z[, intensity]).
        # The last point to land in a voxel is kept; a single representative is
        # enough at this resolution.
        self._voxel_points = {}
        self._use_intensity = None
        self._latest_stamp = 0.0
        self._saved = False

        rospy.Subscriber(
            self.cloud_topic, PointCloud2, self._cloud_callback, queue_size=1,
        )
        rospy.Subscriber(
            self.status_topic, String, self._status_callback, queue_size=5,
        )
        rospy.loginfo(
            "save_pcd: merging %s into %s (voxel %.2f m)",
            self.cloud_topic, self.pcd_dir, self.voxel_size,
        )

        # Mirror FAST-LIO's native behaviour: on Ctrl-C (SIGINT) or any ROS
        # shutdown, dump whatever has accumulated so far.
        rospy.on_shutdown(self._on_shutdown)

    def _cloud_callback(self, msg):
        self._merge(msg)

    def _merge(self, cloud):
        field_names = [field.name for field in cloud.fields]
        use_intensity = "intensity" in field_names
        if self._use_intensity is None:
            self._use_intensity = use_intensity
        names = ("x", "y", "z", "intensity") if use_intensity else ("x", "y", "z")

        inv = 1.0 / self.voxel_size
        points = self._voxel_points
        for point in pc2.read_points(cloud, field_names=names, skip_nans=True):
            key = (
                int(math.floor(point[0] * inv)),
                int(math.floor(point[1] * inv)),
                int(math.floor(point[2] * inv)),
            )
            points[key] = point

        self._latest_stamp = cloud.header.stamp.to_sec()

    def _status_callback(self, msg):
        if self._saved:
            return
        if COMPLETION_MARKER not in msg.data:
            return
        if not self._voxel_points:
            rospy.logwarn("completion seen but no cloud received yet")
            return
        self._save()
        self._saved = True

    def _on_shutdown(self):
        """Save the accumulated map on Ctrl-C / ROS shutdown."""
        if self._saved:
            return
        if not self._voxel_points:
            rospy.logwarn("shutdown before any cloud was received; nothing to save")
            return
        self._save(interrupted=True)
        self._saved = True

    def _save(self, interrupted=False):
        os.makedirs(self.pcd_dir, exist_ok=True)
        stamp = self._latest_stamp
        if stamp <= 0.0:
            stamp = rospy.get_time()
        suffix = "_interrupted" if interrupted else ""
        path = os.path.join(self.pcd_dir, "full_map_%.3f%s.pcd" % (stamp, suffix))
        self._write_ascii_pcd(self._voxel_points, self._use_intensity, path)
        rospy.loginfo(
            "saved accumulated cloud (%d points) to %s",
            len(self._voxel_points), path,
        )

    @staticmethod
    def _write_ascii_pcd(voxel_points, use_intensity, path):
        n = len(voxel_points)
        if use_intensity:
            header = (
                "# .PCD v0.7 - Point Cloud Data file format\n"
                "VERSION 0.7\n"
                "FIELDS x y z intensity\n"
                "SIZE 4 4 4 4\n"
                "TYPE F F F F\n"
                "COUNT 1 1 1 1\n"
                "WIDTH %d\n"
                "HEIGHT 1\n"
                "VIEWPOINT 0 0 0 1 0 0 0\n"
                "POINTS %d\n"
                "DATA ascii\n"
            ) % (n, n)
        else:
            header = (
                "# .PCD v0.7 - Point Cloud Data file format\n"
                "VERSION 0.7\n"
                "FIELDS x y z\n"
                "SIZE 4 4 4\n"
                "TYPE F F F\n"
                "COUNT 1 1 1\n"
                "WIDTH %d\n"
                "HEIGHT 1\n"
                "VIEWPOINT 0 0 0 1 0 0 0\n"
                "POINTS %d\n"
                "DATA ascii\n"
            ) % (n, n)

        lines = [
            " ".join("%f" % value for value in point)
            for point in voxel_points.values()
        ]
        with open(path, "w") as stream:
            stream.write(header)
            stream.write("\n".join(lines))
            stream.write("\n")


def main():
    rospy.init_node("save_pcd")
    PcdSaver()
    rospy.spin()


if __name__ == "__main__":
    main()
