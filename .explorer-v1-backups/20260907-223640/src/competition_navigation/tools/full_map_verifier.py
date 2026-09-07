#!/usr/bin/env python3
"""Independent full-map acceptance monitor for the competition world."""

from __future__ import print_function

import json
import os

import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String


class FullMapVerifier(object):
    def __init__(self):
        self.metadata_path = rospy.get_param(
            "~layout_metadata_path",
            "/home/uf/SimEnv/generated_building/layout_metadata.json",
        )
        self.report_path = rospy.get_param(
            "~report_path",
            "/home/uf/SimEnv/results/full_map_coverage.json",
        )
        self.min_floor_coverage = float(rospy.get_param("~min_floor_coverage", 0.45))
        self.min_room_coverage = float(rospy.get_param("~min_room_coverage", 0.70))
        with open(self.metadata_path, "r") as stream:
            self.metadata = json.load(stream)
        self.floors = self.metadata.get("floors", [])
        if not self.floors:
            raise RuntimeError("layout metadata contains no floors")
        self.maps = {}
        self.status = "waiting"
        for floor_index in range(len(self.floors)):
            rospy.Subscriber(
                "/competition_navigation/map_floor_%d" % floor_index,
                OccupancyGrid,
                self._map_callback,
                callback_args=floor_index,
                queue_size=1,
            )
        rospy.Subscriber(
            "/competition_navigation/status", String, self._status_callback, queue_size=2
        )
        rospy.Timer(rospy.Duration(5.0), self._report_timer)
        rospy.loginfo("full-map verifier ready for %d floors", len(self.floors))

    def _map_callback(self, message, floor_index):
        self.maps[floor_index] = message

    def _status_callback(self, message):
        self.status = message.data

    @staticmethod
    def _parse_room_tasks(status):
        marker = "room_tasks="
        if marker not in status:
            return []
        payload = status.split(marker, 1)[1].split()[0]
        if payload == "none":
            return []
        tasks = []
        for record in payload.split(","):
            fields = record.split(":")
            if len(fields) != 9:
                continue
            try:
                tasks.append({
                    "floor": int(fields[0]),
                    "node_id": int(fields[1]),
                    "side": fields[2],
                    "y": float(fields[3]),
                    "entered": bool(int(fields[4])),
                    "interior_observation": bool(int(fields[5])),
                    "completed": bool(int(fields[6])),
                    "blocked": bool(int(fields[7])),
                    "coverage": float(fields[8]),
                })
            except (TypeError, ValueError):
                continue
        return tasks

    @staticmethod
    def _match_room_task(room, floor_index, tasks):
        door_pose = room.get("door_pose", [])
        if len(door_pose) < 2:
            return None
        side = "L" if float(door_pose[0]) < 0.0 else "R"
        candidates = [
            task for task in tasks
            if task["floor"] == floor_index and task["side"] == side
        ]
        if not candidates:
            return None
        nearest = min(candidates, key=lambda task: abs(task["y"] - float(door_pose[1])))
        return nearest if abs(nearest["y"] - float(door_pose[1])) <= 1.0 else None

    @staticmethod
    def _coverage_in_bounds(message, bounds):
        values = np.asarray(message.data, dtype=np.int16).reshape(
            message.info.height, message.info.width
        )
        resolution = float(message.info.resolution)
        origin_x = float(message.info.origin.position.x)
        origin_y = float(message.info.origin.position.y)
        x0 = max(0, int(np.floor((float(bounds["x_min"]) - origin_x) / resolution)))
        x1 = min(
            message.info.width,
            int(np.ceil((float(bounds["x_max"]) - origin_x) / resolution)) + 1,
        )
        y0 = max(0, int(np.floor((float(bounds["y_min"]) - origin_y) / resolution)))
        y1 = min(
            message.info.height,
            int(np.ceil((float(bounds["y_max"]) - origin_y) / resolution)) + 1,
        )
        if x1 <= x0 or y1 <= y0:
            return 0.0
        region = values[y0:y1, x0:x1]
        return float(np.count_nonzero(region >= 0)) / int(region.size)

    def _build_report(self):
        footprint = self.metadata.get("footprint", {})
        entrance = self.metadata.get("entrance_pose", [0.0, 0.0])
        footprint_bounds = {
            "x_min": float(entrance[0]) - float(footprint["width"]) / 2.0,
            "x_max": float(entrance[0]) + float(footprint["width"]) / 2.0,
            "y_min": float(entrance[1]),
            "y_max": float(entrance[1]) + float(footprint["length"]),
        }
        floor_results = []
        room_results = []
        room_tasks = self._parse_room_tasks(self.status)
        for floor in self.floors:
            floor_index = int(floor["floor_index"])
            message = self.maps.get(floor_index)
            floor_ratio = (
                self._coverage_in_bounds(message, footprint_bounds) if message else 0.0
            )
            floor_results.append(
                {
                    "floor": floor_index,
                    "known_ratio": floor_ratio,
                    "passed": floor_ratio >= self.min_floor_coverage,
                }
            )
            for room in floor.get("rooms", []):
                room_ratio = (
                    self._coverage_in_bounds(message, room["bounds"]) if message else 0.0
                )
                task = self._match_room_task(room, floor_index, room_tasks)
                task_passed = bool(
                    task and task["entered"] and
                    task["interior_observation"] and
                    task["completed"] and not task["blocked"] and
                    task["coverage"] >= self.min_room_coverage
                )
                room_results.append(
                    {
                        "id": room["id"],
                        "floor": floor_index,
                        "known_ratio": room_ratio,
                        "task_found": task is not None,
                        "entered": bool(task and task["entered"]),
                        "interior_observation": bool(
                            task and task["interior_observation"]
                        ),
                        "completed": bool(task and task["completed"]),
                        "blocked": bool(task and task["blocked"]),
                        "task_coverage": 0.0 if task is None else task["coverage"],
                        "passed": (
                            room_ratio >= self.min_room_coverage and task_passed
                        ),
                    }
                )
        navigation_complete = "returned_home full_map=True" in self.status
        passed = (
            navigation_complete
            and all(item["passed"] for item in floor_results)
            and all(item["passed"] for item in room_results)
        )
        return {
            "passed": passed,
            "navigation_complete": navigation_complete,
            "min_floor_coverage": self.min_floor_coverage,
            "min_room_coverage": self.min_room_coverage,
            "floors": floor_results,
            "rooms": room_results,
            "room_tasks": room_tasks,
            "status": self.status,
        }

    def _write_report(self, report):
        directory = os.path.dirname(self.report_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        temporary_path = self.report_path + ".tmp"
        with open(temporary_path, "w") as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary_path, self.report_path)

    def _report_timer(self, _event):
        report = self._build_report()
        self._write_report(report)
        floor_text = ",".join(
            "%d:%.3f" % (item["floor"], item["known_ratio"])
            for item in report["floors"]
        )
        room_passed = sum(1 for item in report["rooms"] if item["passed"])
        rospy.loginfo_throttle(
            10.0,
            "full-map coverage floors=[%s] rooms=%d/%d complete=%s passed=%s",
            floor_text,
            room_passed,
            len(report["rooms"]),
            report["navigation_complete"],
            report["passed"],
        )
        if report["navigation_complete"]:
            if report["passed"]:
                rospy.loginfo("FULL_MAP_ACCEPTANCE_PASS report=%s", self.report_path)
            else:
                rospy.logerr("FULL_MAP_ACCEPTANCE_FAIL report=%s", self.report_path)
            rospy.signal_shutdown("full-map verification completed")


def main():
    rospy.init_node("full_map_verifier")
    FullMapVerifier()
    rospy.spin()


if __name__ == "__main__":
    main()
