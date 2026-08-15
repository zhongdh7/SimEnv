#!/usr/bin/env python3
"""Save the FAST-LIO accumulated point cloud to PCD after exploration finishes.

Subscribes to ``/cloud_registered`` and ``/competition_navigation/status``.
When the status reports ``returned_home full_map=True``, writes the latest
accumulated cloud as an ASCII PCD file under ``<fastlio_slam_merge>/PCD/``.
"""

from __future__ import print_function

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

        self._latest_cloud = None
        self._saved = False

        rospy.Subscriber(
            self.cloud_topic, PointCloud2, self._cloud_callback, queue_size=1,
        )
        rospy.Subscriber(
            self.status_topic, String, self._status_callback, queue_size=5,
        )
        rospy.loginfo(
            "save_pcd: watching %s, will save to %s",
            self.status_topic, self.pcd_dir,
        )

        # FAST-LIO's native node dumps the accumulated map when it is
        # interrupted (SIGINT -> save PCD after the main loop exits).  Mirror
        # that behaviour: on Ctrl-C, or any other ROS shutdown, save whatever
        # cloud has accumulated so far even if full-map completion never fired.
        rospy.on_shutdown(self._on_shutdown)

    def _cloud_callback(self, msg):
        self._latest_cloud = msg

    def _status_callback(self, msg):
        if self._saved:
            return
        if COMPLETION_MARKER not in msg.data:
            return
        if self._latest_cloud is None:
            rospy.logwarn("completion seen but no cloud received yet")
            return
        self._save(self._latest_cloud)
        self._saved = True

    def _on_shutdown(self):
        """Save the accumulated cloud on Ctrl-C / ROS shutdown."""
        if self._saved:
            return
        if self._latest_cloud is None:
            rospy.logwarn("shutdown before any cloud was received; nothing to save")
            return
        self._save(self._latest_cloud, interrupted=True)
        self._saved = True

    def _save(self, cloud, interrupted=False):
        os.makedirs(self.pcd_dir, exist_ok=True)
        stamp = cloud.header.stamp.to_sec()
        if stamp <= 0.0:
            stamp = rospy.get_time()
        suffix = "_interrupted" if interrupted else ""
        path = os.path.join(self.pcd_dir, "full_map_%.3f%s.pcd" % (stamp, suffix))
        self._write_ascii_pcd(cloud, path)
        rospy.loginfo("saved accumulated cloud to %s", path)

    @staticmethod
    def _write_ascii_pcd(cloud, path):
        field_names = [field.name for field in cloud.fields]
        use_intensity = "intensity" in field_names
        names = ("x", "y", "z", "intensity") if use_intensity else ("x", "y", "z")
        points = list(pc2.read_points(cloud, field_names=names, skip_nans=True))

        if use_intensity:
            fields = "FIELDS x y z intensity\n"
            sizes = "SIZE 4 4 4 4\n"
            types = "TYPE F F F F\n"
            counts = "COUNT 1 1 1 1\n"
        else:
            fields = "FIELDS x y z\n"
            sizes = "SIZE 4 4 4\n"
            types = "TYPE F F F\n"
            counts = "COUNT 1 1 1\n"

        with open(path, "w") as stream:
            stream.write("# .PCD v0.7 - Point Cloud Data file format\n")
            stream.write("VERSION 0.7\n")
            stream.write(fields)
            stream.write(sizes)
            stream.write(types)
            stream.write(counts)
            stream.write("WIDTH %d\n" % len(points))
            stream.write("HEIGHT 1\n")
            stream.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            stream.write("POINTS %d\n" % len(points))
            stream.write("DATA ascii\n")
            for point in points:
                stream.write(" ".join("%f" % value for value in point) + "\n")


def main():
    rospy.init_node("save_pcd")
    PcdSaver()
    rospy.spin()


if __name__ == "__main__":
    main()
