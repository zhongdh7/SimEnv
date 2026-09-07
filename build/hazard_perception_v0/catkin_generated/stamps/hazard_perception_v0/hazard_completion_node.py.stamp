#!/usr/bin/env python3
"""Finalize confirmed hazards and export them in the sensor map frame.

This node is an output-side coordinator only.  It observes the public
navigation completion status, asks the production hazard pipeline to finalize
its confirmed tracks, and converts the persisted start-frame coordinates using
the transform captured by that same pipeline.  It never reads Gazebo model
states or evaluator truth.
"""

from __future__ import annotations

import glob
import json
import math
import os
import threading
import time

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


COMPLETION_MARKER = "returned_home full_map=True"


def _finite(values):
    return all(math.isfinite(float(value)) for value in values)


def _rotate(quaternion, vector):
    x, y, z, w = [float(value) for value in quaternion]
    vx, vy, vz = [float(value) for value in vector]
    qvec = (x, y, z)
    uv = (
        qvec[1] * vz - qvec[2] * vy,
        qvec[2] * vx - qvec[0] * vz,
        qvec[0] * vy - qvec[1] * vx,
    )
    uuv = (
        qvec[1] * uv[2] - qvec[2] * uv[1],
        qvec[2] * uv[0] - qvec[0] * uv[2],
        qvec[0] * uv[1] - qvec[1] * uv[0],
    )
    return tuple(
        vector[index] + 2.0 * (w * uv[index] + uuv[index])
        for index in range(3)
    )


def transform_point(transform, point):
    """Apply a persisted parent<-child transform to a child-frame point."""
    translation = transform["translation"]
    quaternion = transform["quaternion_xyzw"]
    rotated = _rotate(
        (
            quaternion["x"],
            quaternion["y"],
            quaternion["z"],
            quaternion["w"],
        ),
        point,
    )
    result = (
        rotated[0] + float(translation["x"]),
        rotated[1] + float(translation["y"]),
        rotated[2] + float(translation["z"]),
    )
    if not _finite(result):
        raise ValueError("transformed hazard point contains NaN/Inf")
    return result


def _write_json(path, payload):
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    temporary = path + ".tmp"
    with open(temporary, "w", encoding="utf-8") as output:
        json.dump(payload, output, ensure_ascii=False, indent=2, sort_keys=True)
        output.write("\n")
    os.replace(temporary, path)


class HazardCompletionNode(object):
    def __init__(self):
        self.status_topic = rospy.get_param(
            "~status_topic", "/competition_navigation/status"
        )
        self.finalize_service = rospy.get_param(
            "~finalize_service", "/hazard_perception_v0/finalize"
        )
        self.output_directory = os.path.abspath(os.path.expanduser(
            rospy.get_param("~output_directory", "~/SimEnv/results/hazard_perception_v0")
        ))
        self.run_id = str(rospy.get_param("~run_id", "")).strip()
        self.map_frame = str(rospy.get_param("~map_frame", "odom"))
        self.manifest_path = os.path.abspath(os.path.expanduser(
            rospy.get_param(
                "~manifest_path",
                os.path.join(self.output_directory, "run_manifest.json"),
            )
        ))
        self.map_hazards_path = os.path.abspath(os.path.expanduser(
            rospy.get_param(
                "~map_hazards_path",
                os.path.join(self.output_directory, "hazards_in_map.json"),
            )
        ))
        self.navigation_report_path = os.path.abspath(os.path.expanduser(
            rospy.get_param("~navigation_report_path", "")
        )) if rospy.get_param("~navigation_report_path", "") else ""
        self.pcd_directory = os.path.abspath(os.path.expanduser(
            rospy.get_param("~pcd_directory", "")
        )) if rospy.get_param("~pcd_directory", "") else ""
        self.map_directory = os.path.abspath(os.path.expanduser(
            rospy.get_param("~map_directory", "")
        )) if rospy.get_param("~map_directory", "") else ""
        self.map_wait_seconds = max(
            0.0, float(rospy.get_param("~map_wait_seconds", 20.0))
        )
        self.marker_topic = rospy.get_param(
            "~marker_topic", "/hazard_perception_v0/map_markers"
        )

        self._lock = threading.Lock()
        self._worker_started = False
        self._finalized = False
        self._last_error = ""
        self.marker_publisher = rospy.Publisher(
            self.marker_topic, MarkerArray, queue_size=1, latch=True
        )
        rospy.Subscriber(self.status_topic, String, self._status_callback, queue_size=5)
        rospy.loginfo(
            "hazard_completion: status=%s finalize=%s map_frame=%s output=%s",
            self.status_topic,
            self.finalize_service,
            self.map_frame,
            self.output_directory,
        )

    def _status_callback(self, message):
        if COMPLETION_MARKER not in message.data:
            return
        with self._lock:
            if self._worker_started:
                return
            self._worker_started = True
        threading.Thread(
            target=self._complete,
            args=(message.data,),
            name="hazard-completion",
            daemon=True,
        ).start()

    def _find_hazard_json(self):
        pattern = os.path.join(self.output_directory, "hazards_*.json")
        paths = [
            path for path in glob.glob(pattern)
            if not path.endswith(".tmp")
            and (not self.run_id or ("_" + self.run_id + ".json") in path)
        ]
        if not paths:
            return ""
        return max(paths, key=os.path.getmtime)

    def _find_final_pcd(self):
        if not self.pcd_directory:
            return ""
        deadline = time.time() + self.map_wait_seconds
        while time.time() <= deadline and not rospy.is_shutdown():
            paths = [
                path for path in glob.glob(
                    os.path.join(self.pcd_directory, "full_map_*.pcd")
                )
                if "_interrupted.pcd" not in path
            ]
            if paths:
                return max(paths, key=os.path.getmtime)
            time.sleep(0.25)
        return ""

    def _find_final_map(self):
        if not self.map_directory:
            return ""
        deadline = time.time() + self.map_wait_seconds
        while time.time() <= deadline and not rospy.is_shutdown():
            paths = [
                path for path in glob.glob(
                    os.path.join(self.map_directory, "*_carto_map_*.pgm")
                )
                if "_interrupted.pgm" not in path
                and os.path.isfile(os.path.splitext(path)[0] + ".yaml")
            ]
            if paths:
                return max(paths, key=os.path.getmtime)
            time.sleep(0.25)
        return ""

    def _load_navigation_report(self):
        if not self.navigation_report_path:
            raise RuntimeError("navigation report path is not configured")
        deadline = time.time() + self.map_wait_seconds
        while time.time() <= deadline and not rospy.is_shutdown():
            if os.path.isfile(self.navigation_report_path):
                with open(
                        self.navigation_report_path, encoding="utf-8") as source:
                    report = json.load(source)
                if report.get("passed") is True:
                    return report
            time.sleep(0.25)
        raise RuntimeError(
            "navigation report did not reach passed=true: %s"
            % self.navigation_report_path
        )

    def _finalize_hazard(self):
        rospy.wait_for_service(self.finalize_service, timeout=15.0)
        response = rospy.ServiceProxy(self.finalize_service, Trigger)()
        if not response.success:
            raise RuntimeError(response.message)
        deadline = time.time() + 10.0
        path = ""
        while time.time() <= deadline and not rospy.is_shutdown():
            path = self._find_hazard_json()
            if path:
                break
            time.sleep(0.1)
        if not path:
            raise RuntimeError("hazard finalize returned no JSON result")
        return path, response.message

    def _load_and_transform(self, source_path):
        with open(source_path, encoding="utf-8") as source:
            payload = json.load(source)
        source_frame = str(payload.get("frame_id", ""))
        transform = payload.get("start_frame_transform")
        if source_frame == self.map_frame:
            transformed = payload["hazards"]
        else:
            if not transform:
                raise ValueError("hazard result has no start-frame transform")
            if str(transform.get("parent_frame")) != self.map_frame:
                raise ValueError(
                    "hazard transform parent %s != %s" % (
                        transform.get("parent_frame"), self.map_frame
                    )
                )
            transformed = []
            for hazard in payload.get("hazards", []):
                point = transform_point(
                    transform,
                    (hazard["x"], hazard["y"], hazard["z"]),
                )
                record = dict(hazard)
                record["x"], record["y"], record["z"] = point
                record["source_frame"] = source_frame
                transformed.append(record)
        return payload, transformed

    def _publish_markers(self, hazards):
        markers = MarkerArray()
        for index, hazard in enumerate(hazards):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = "confirmed_hazards_map"
            marker.id = int(hazard.get("id", index))
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(hazard["x"])
            marker.pose.position.y = float(hazard["y"])
            marker.pose.position.z = float(hazard["z"])
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.30
            marker.color.r = 1.0
            marker.color.g = 0.1
            marker.color.b = 0.1
            marker.color.a = 0.95
            markers.markers.append(marker)
        self.marker_publisher.publish(markers)

    def _complete(self, completion_status):
        manifest = {
            "schema": "v0_sensor_only_run_manifest_v1",
            "completion_status": completion_status,
            "map_frame": self.map_frame,
            "run_id": self.run_id,
            "passed": False,
        }
        try:
            hazard_path, finalize_message = self._finalize_hazard()
            payload, hazards = self._load_and_transform(hazard_path)
            _write_json(self.map_hazards_path, {
                "schema": "v0_sensor_only_hazards_map_v1",
                "frame_id": self.map_frame,
                "source_path": hazard_path,
                "hazard_count": len(hazards),
                "hazards": hazards,
            })
            self._publish_markers(hazards)
            pcd_path = self._find_final_pcd()
            if not pcd_path:
                raise RuntimeError("sensor 3-D map PCD was not saved")
            map_path = self._find_final_map()
            if not map_path:
                raise RuntimeError("sensor 2-D SLAM map was not saved")
            navigation_report = self._load_navigation_report()
            manifest.update({
                "passed": True,
                "hazard_finalize_message": finalize_message,
                "hazard_result_path": hazard_path,
                "hazards_in_map_path": self.map_hazards_path,
                "hazard_count": int(payload.get("hazard_count", len(hazards))),
                "pcd_path": pcd_path,
                "map_path": map_path,
                "map_yaml_path": os.path.splitext(map_path)[0] + ".yaml",
                "navigation_report_path": self.navigation_report_path,
                "navigation_report": navigation_report,
            })
        except Exception as error:
            self._last_error = str(error)
            manifest["error"] = self._last_error
            rospy.logerr("hazard_completion failed: %s", error)
        _write_json(self.manifest_path, manifest)
        with self._lock:
            self._finalized = bool(manifest["passed"])
        if manifest["passed"]:
            rospy.loginfo(
                "hazard_completion passed: hazards=%d map=%s manifest=%s",
                manifest["hazard_count"],
                self.map_hazards_path,
                self.manifest_path,
            )


def main():
    rospy.init_node("hazard_completion")
    HazardCompletionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
