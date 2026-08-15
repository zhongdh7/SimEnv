#!/usr/bin/env python3
"""Save a drift-free accumulated point cloud to PCD after exploration finishes.

Subscribes to FAST-LIO's ``/cloud_registered_body`` (lidar points expressed in
the IMU ``body`` frame) and ``/competition_navigation/status``.

FAST-LIO's world-frame ``/cloud_registered`` drifts over long runs because LIO
has no loop closure, which smears the map.  ``/cloud_registered_body`` is
drift-free per frame (motion compensation is accurate over a 10 Hz scan), so
this node places every frame in the ground-truth ``odom`` frame using the TF
chain ``odom -> base -> imu_link`` (FAST-LIO's ``body`` frame IS the IMU frame
``imu_link``).  The result is a world-anchored map that does not inherit
FAST-LIO's long-term drift.

The merged map is voxel-deduplicated and written to PCD when the status reports
``returned_home full_map=True``, or on Ctrl-C / ROS shutdown.
"""

from __future__ import print_function

import math
import os
import threading

import numpy as np
import rospkg
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf
import tf.transformations as tft
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

COMPLETION_MARKER = "returned_home full_map=True"


class PcdSaver(object):
    def __init__(self):
        self.cloud_topic = rospy.get_param("~cloud_topic", "/cloud_registered_body")
        self.status_topic = rospy.get_param(
            "~status_topic", "/competition_navigation/status"
        )
        self.pcd_dir = rospy.get_param("~pcd_dir", "")
        if not self.pcd_dir:
            self.pcd_dir = os.path.join(
                rospkg.RosPack().get_path("fastlio_slam_merge"), "PCD"
            )
        # Ground-truth frame the map is anchored to, and the frame of the
        # /cloud_registered_body points.  FAST-LIO calls its IMU frame "body";
        # that frame is the robot's ``imu_link``, so the world -> body transform
        # is looked up as ``<map_frame> -> <body_frame>``.
        self.map_frame = rospy.get_param("~map_frame", "odom")
        self.body_frame = rospy.get_param("~body_frame", "imu_link")
        # Voxel size (m) used to deduplicate the merged cloud.
        self.voxel_size = float(rospy.get_param("~voxel_size", 0.2))

        self._voxel_points = {}  # (ix, iy, iz) -> (x, y, z[, intensity])
        self._use_intensity = None
        self._latest_stamp = 0.0
        self._saved = False
        self._lock = threading.Lock()
        self._tf_listener = tf.TransformListener(cache_time=rospy.Duration(10.0))

        rospy.Subscriber(
            self.cloud_topic, PointCloud2, self._cloud_callback, queue_size=1,
        )
        rospy.Subscriber(
            self.status_topic, String, self._status_callback, queue_size=5,
        )
        rospy.loginfo(
            "save_pcd: anchoring %s -> %s via TF %s -> %s (voxel %.2f m)",
            self.cloud_topic, self.pcd_dir,
            self.map_frame, self.body_frame, self.voxel_size,
        )

        # Mirror FAST-LIO's native behaviour: on Ctrl-C (SIGINT) or any ROS
        # shutdown, dump whatever has accumulated so far.
        rospy.on_shutdown(self._on_shutdown)

    def _cloud_callback(self, msg):
        self._merge(msg)

    def _merge(self, cloud):
        try:
            translation, rotation = self._tf_listener.lookupTransform(
                self.map_frame, self.body_frame, cloud.header.stamp
            )
        except tf.ExtrapolationException:
            translation, rotation = self._tf_listener.lookupTransform(
                self.map_frame, self.body_frame, rospy.Time(0)
            )
        except (tf.LookupException, tf.ConnectivityException):
            rospy.logwarn_throttle(
                5.0, "TF unavailable: %s -> %s", self.map_frame, self.body_frame
            )
            return

        matrix = tft.quaternion_matrix(rotation)
        matrix[0:3, 3] = translation

        field_names = [field.name for field in cloud.fields]
        use_intensity = "intensity" in field_names
        if self._use_intensity is None:
            self._use_intensity = use_intensity

        inv = 1.0 / self.voxel_size
        new_points = {}
        if use_intensity:
            for x, y, z, intensity in pc2.read_points(
                    cloud, field_names=("x", "y", "z", "intensity"),
                    skip_nans=True):
                wx, wy, wz = matrix.dot(np.array([x, y, z, 1.0]))[:3]
                key = (
                    int(math.floor(wx * inv)),
                    int(math.floor(wy * inv)),
                    int(math.floor(wz * inv)),
                )
                new_points[key] = (float(wx), float(wy), float(wz), float(intensity))
        else:
            for x, y, z in pc2.read_points(
                    cloud, field_names=("x", "y", "z"), skip_nans=True):
                wx, wy, wz = matrix.dot(np.array([x, y, z, 1.0]))[:3]
                key = (
                    int(math.floor(wx * inv)),
                    int(math.floor(wy * inv)),
                    int(math.floor(wz * inv)),
                )
                new_points[key] = (float(wx), float(wy), float(wz))

        with self._lock:
            self._voxel_points.update(new_points)
        self._latest_stamp = cloud.header.stamp.to_sec()

    def _status_callback(self, msg):
        if self._saved:
            return
        if COMPLETION_MARKER not in msg.data:
            return
        with self._lock:
            if not self._voxel_points:
                rospy.logwarn("completion seen but no cloud received yet")
                return
        self._save()
        self._saved = True

    def _on_shutdown(self):
        """Save the accumulated map on Ctrl-C / ROS shutdown."""
        if self._saved:
            return
        with self._lock:
            if not self._voxel_points:
                rospy.logwarn("shutdown before any cloud was received; nothing to save")
                return
        self._save(interrupted=True)
        self._saved = True

    def _save(self, interrupted=False):
        # Snapshot the merged points under the lock so the (slow) file write
        # never races with _cloud_callback mutating the dict.
        with self._lock:
            snapshot = list(self._voxel_points.values())
            use_intensity = self._use_intensity
        os.makedirs(self.pcd_dir, exist_ok=True)
        stamp = self._latest_stamp
        if stamp <= 0.0:
            stamp = rospy.get_time()
        suffix = "_interrupted" if interrupted else ""
        path = os.path.join(self.pcd_dir, "full_map_%.3f%s.pcd" % (stamp, suffix))
        self._write_ascii_pcd(snapshot, use_intensity, path)
        rospy.loginfo("saved accumulated cloud (%d points) to %s", len(snapshot), path)

    @staticmethod
    def _write_ascii_pcd(points, use_intensity, path):
        n = len(points)
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
            " ".join("%f" % value for value in point) for point in points
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
