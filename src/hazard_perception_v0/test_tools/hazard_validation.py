#!/usr/bin/env python3
"""Gazebo-only black-box validation for the RGB-D hazard pipeline.

This node is deliberately kept under test_tools.  It may read Gazebo ground
truth, but it only observes the production pipeline through its public ROS
topics.  No ground-truth data is published to the production node.
"""

import csv
import json
import math
import os
import re
import signal
import threading
import time
from collections import deque
from datetime import datetime

import cv2
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import (
    DeleteModel,
    GetModelState,
    GetWorldProperties,
    SetModelState,
    SpawnModel,
)
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf.transformations import quaternion_from_matrix, quaternion_matrix

from hazard_perception_v0.msg import Hazard3DArray, HazardDetection2DArray


CSV_FIELDS = [
    "case_id", "case_type", "model_name", "shape", "distance_m", "angle_deg",
    "expected_hazard", "two_d_detected", "three_d_available", "detected",
    "success", "source", "stable", "two_d_sample_count", "sample_count",
    "matched_track_id", "truth_x", "truth_y", "truth_z", "mean_x", "mean_y",
    "mean_z", "mean_error_m", "rmse_m", "max_error_m", "std_x_m", "std_y_m",
    "std_z_m", "std_norm_m", "unique_target_track_count",
    "duplicate_target_count", "false_positive_count", "notes",
]


def _pose_to_matrix(pose):
    q = pose.orientation
    matrix = quaternion_matrix([q.x, q.y, q.z, q.w])
    matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return matrix


def _transform_to_matrix(transform):
    q = transform.rotation
    matrix = quaternion_matrix([q.x, q.y, q.z, q.w])
    matrix[:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
    return matrix


def _matrix_to_pose(matrix):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = matrix[:3, 3].tolist()
    q = quaternion_from_matrix(matrix)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q.tolist()
    return pose


def _xyz(vector):
    return {"x": float(vector[0]), "y": float(vector[1]), "z": float(vector[2])}


def compute_sample_metrics(samples, truth):
    """Return localization and stability metrics for Nx3 samples."""
    points = np.asarray(samples, dtype=np.float64).reshape((-1, 3))
    truth = np.asarray(truth, dtype=np.float64).reshape(3)
    if points.size == 0:
        return {
            "sample_count": 0, "mean_position": None, "mean_error_m": None,
            "rmse_m": None, "max_error_m": None, "std_xyz_m": None,
            "std_norm_m": None,
        }
    errors = np.linalg.norm(points - truth, axis=1)
    std = np.std(points, axis=0)
    return {
        "sample_count": int(points.shape[0]),
        "mean_position": _xyz(np.mean(points, axis=0)),
        "mean_error_m": float(np.mean(errors)),
        "rmse_m": float(np.sqrt(np.mean(errors ** 2))),
        "max_error_m": float(np.max(errors)),
        "std_xyz_m": _xyz(std),
        "std_norm_m": float(np.linalg.norm(std)),
    }


def summarize_cases(cases):
    positives = [case for case in cases if case["expected_hazard"]]
    negatives = [case for case in cases if not case["expected_hazard"]]
    successful = [case for case in positives if case["success"]]
    errors = [case["mean_error_m"] for case in successful if case["mean_error_m"] is not None]
    stds = [case["std_norm_m"] for case in successful if case["std_norm_m"] is not None]
    false_positive_cases = [case for case in negatives if case["false_positive_count"] > 0]
    two_d_successes = [case for case in positives if case.get("two_d_detected", False)]
    available_3d = [case for case in positives if case.get("three_d_available", False)]
    source_counts = {name: 0 for name in ("rgbd", "livox", "fused", "monocular", "2d_only")}
    positive_source_counts = {name: 0 for name in source_counts}
    for case in cases:
        source = case.get("source", "2d_only")
        source_counts[source] = source_counts.get(source, 0) + 1
        if case["expected_hazard"]:
            positive_source_counts[source] = positive_source_counts.get(source, 0) + 1
    return {
        "positive_case_count": len(positives),
        "two_d_successful_positive_cases": len(two_d_successes),
        "two_d_detection_success_rate": (
            float(len(two_d_successes) / len(positives)) if positives else None
        ),
        "three_d_available_positive_cases": len(available_3d),
        "three_d_data_availability_rate": (
            float(len(available_3d) / len(positives)) if positives else None
        ),
        "successful_positive_cases": len(successful),
        "three_d_localization_success_rate": (
            float(len(successful) / len(positives)) if positives else None
        ),
        "detection_success_rate": float(len(successful) / len(positives)) if positives else None,
        "mean_localization_error_m": float(np.mean(errors)) if errors else None,
        "localization_rmse_m": float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
        "max_localization_error_m": float(np.max(errors)) if errors else None,
        "mean_coordinate_std_norm_m": float(np.mean(stds)) if stds else None,
        "duplicate_target_count": int(sum(case["duplicate_target_count"] for case in positives)),
        "negative_case_count": len(negatives),
        "false_positive_case_count": len(false_positive_cases),
        "false_positive_case_rate": (
            float(len(false_positive_cases) / len(negatives)) if negatives else None
        ),
        "false_positive_detection_count": int(
            sum(case["false_positive_count"] for case in negatives)
        ),
        "source_case_counts": source_counts,
        "positive_source_case_counts": positive_source_counts,
    }


class HazardValidationNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.hazard_messages = deque(maxlen=4000)
        self.latest_hazard = None
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_debug = None
        self.latest_camera_info = None
        self.detection_messages = deque(maxlen=4000)
        self.diagnostic_messages = deque(maxlen=4000)
        self.latest_detection = None
        self.latest_diagnostic = None
        self.original_states = {}
        self.spawned_models = set()
        self.managed_models = []
        self.restored = False
        self.interrupted = False
        self._load_parameters()

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(60.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self._subscribers = [
            rospy.Subscriber(self.hazards_topic, Hazard3DArray, self._hazard_callback, queue_size=100),
            rospy.Subscriber(self.rgb_topic, Image, self._rgb_callback, queue_size=1, buff_size=2**24),
            rospy.Subscriber(self.depth_topic, Image, self._depth_callback, queue_size=1, buff_size=2**24),
            rospy.Subscriber(self.debug_topic, Image, self._debug_callback, queue_size=1, buff_size=2**24),
            rospy.Subscriber(self.camera_info_topic, CameraInfo, self._camera_info_callback, queue_size=1),
            rospy.Subscriber(
                self.detections_topic, HazardDetection2DArray,
                self._detection_callback, queue_size=100,
            ),
            rospy.Subscriber(
                self.geometry_diagnostics_topic, String,
                self._diagnostic_callback, queue_size=100,
            ),
        ]

        self._wait_for_services()
        self.get_world = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
        self.get_model = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.set_model = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

        self.run_directory = self._make_run_directory()
        rospy.on_shutdown(self.restore_scene)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _load_parameters(self):
        get = rospy.get_param
        self.distances = [float(value) for value in get("~distances_m", [1, 2, 3, 4, 5, 6])]
        self.angles = [float(value) for value in get("~angles_deg", [0, 15, -15, 30, -30])]
        self.interferences = list(get("~interferences", []))
        self.target_model_param = str(get("~target_model", "")).strip()
        self.target_model_regex = str(get("~target_model_regex", r"^danger_red_sphere"))
        self.managed_model_regex = str(
            get("~managed_model_regex", r"^(danger_red_sphere|distractor_red_(box|cylinder))")
        )
        self.robot_model_param = str(get("~robot_model", "")).strip()
        self.robot_model_regex = str(get("~robot_model_regex", r"(^a1_gazebo$|robot)"))
        self.base_frame = str(get("~base_frame", "base"))
        self.camera_frame_param = str(get("~camera_frame", "")).strip()
        self.output_frame_param = str(get("~output_frame", "")).strip()
        self.world_frame = str(get("~world_frame", "world"))
        self.hazards_topic = str(get("~hazards_topic", "/hazard_perception_v0/hazards_3d"))
        self.rgb_topic = str(get("~rgb_topic", "/real_sense/rgb/image_raw"))
        self.depth_topic = str(get("~depth_topic", "/real_sense/depth/image_raw"))
        self.camera_info_topic = str(get("~camera_info_topic", "/real_sense/rgb/camera_info"))
        self.debug_topic = str(get("~debug_topic", "/hazard_perception_v0/pipeline_debug_image"))
        self.detections_topic = str(get("~detections_topic", "/hazard_perception_v0/detections"))
        self.geometry_diagnostics_topic = str(get(
            "~geometry_diagnostics_topic", "/hazard_perception_v0/geometry_diagnostics"
        ))
        self.output_root = os.path.abspath(os.path.expanduser(
            str(get("~output_root", "results/hazard_validation"))
        ))
        self.settle_time = float(get("~settle_time_s", 0.8))
        self.reset_settle_time = float(get("~reset_settle_time_s", 0.35))
        self.stability_timeout = float(get("~stability_timeout_s", 3.0))
        self.stability_window = float(get("~stability_window_s", 0.7))
        self.sample_duration = float(get("~sample_duration_s", 1.0))
        self.min_samples = int(get("~min_samples", 5))
        self.stability_std_threshold = float(get("~stability_std_threshold_m", 0.04))
        self.max_localization_error = float(get("~max_localization_error_m", 0.60))
        self.duplicate_radius = float(get("~duplicate_radius_m", 0.60))
        self.require_confirmed = bool(get("~require_confirmed", True))
        self.target_vertical_offset = float(get("~target_vertical_offset_m", 0.0))
        self.park_distance = float(get("~park_behind_camera_m", 3.0))
        self.service_timeout = float(get("~service_timeout_s", 30.0))
        self.topic_timeout = float(get("~topic_timeout_s", 30.0))
        self.tf_timeout = float(get("~tf_timeout_s", 10.0))
        self.save_npy_depth = bool(get("~save_npy_depth", True))
        self.transition_step = float(get("~transition_step_m", 0.15))
        self.transition_step_time = float(get("~transition_step_time_s", 0.12))
        self.target_track_established = False

        if not self.distances or any(value <= 0.0 for value in self.distances):
            raise ValueError("distances_m must contain positive values")
        if not self.angles or any(abs(value) >= 90.0 for value in self.angles):
            raise ValueError("angles_deg must be non-empty and inside (-90, 90)")
        if any(abs(self.target_vertical_offset) >= value for value in self.distances):
            raise ValueError("abs(target_vertical_offset_m) must be smaller than every distance")
        if (
            self.min_samples < 1 or self.max_localization_error <= 0.0
            or self.transition_step <= 0.0 or self.transition_step_time < 0.0
        ):
            raise ValueError("min_samples and max_localization_error_m must be positive")

    def _wait_for_services(self):
        for service in (
            "/gazebo/get_world_properties", "/gazebo/get_model_state",
            "/gazebo/set_model_state", "/gazebo/spawn_sdf_model", "/gazebo/delete_model",
        ):
            rospy.loginfo("Waiting for %s", service)
            rospy.wait_for_service(service, timeout=self.service_timeout)

    def _make_run_directory(self):
        os.makedirs(self.output_root, exist_ok=True)
        stem = datetime.now().strftime("run_%Y%m%d_%H%M%S_%f")
        path = os.path.join(self.output_root, stem)
        os.makedirs(path)
        return path

    def _signal_handler(self, _signum, _frame):
        self.interrupted = True
        rospy.signal_shutdown("termination signal received")

    def _hazard_callback(self, message):
        received = time.monotonic()
        with self.lock:
            self.latest_hazard = message
            self.hazard_messages.append((received, message))

    def _rgb_callback(self, message):
        with self.lock:
            self.latest_rgb = message

    def _depth_callback(self, message):
        with self.lock:
            self.latest_depth = message

    def _debug_callback(self, message):
        with self.lock:
            self.latest_debug = message

    def _camera_info_callback(self, message):
        with self.lock:
            self.latest_camera_info = message

    def _detection_callback(self, message):
        received = time.monotonic()
        with self.lock:
            self.latest_detection = message
            self.detection_messages.append((received, message))

    def _diagnostic_callback(self, message):
        received = time.monotonic()
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        with self.lock:
            self.latest_diagnostic = payload
            self.diagnostic_messages.append((received, payload))

    def _wait_for_inputs(self):
        deadline = time.monotonic() + self.topic_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self.lock:
                ready = (
                    self.latest_hazard is not None and self.latest_rgb is not None
                    and self.latest_depth is not None and self.latest_camera_info is not None
                    and self.latest_detection is not None and self.latest_diagnostic is not None
                )
            if ready:
                return
            time.sleep(0.05)
        raise RuntimeError(
            "Timed out waiting for RGB, depth, detections, geometry diagnostics and hazards_3d"
        )

    @staticmethod
    def _select_model(names, explicit, regex, description):
        if explicit:
            if explicit not in names:
                raise RuntimeError("Configured {} model {!r} is not in Gazebo".format(description, explicit))
            return explicit
        pattern = re.compile(regex, re.IGNORECASE)
        matches = sorted(name for name in names if pattern.search(name))
        if not matches:
            raise RuntimeError("Cannot auto-discover {} model with regex {!r}".format(description, regex))
        return matches[0]

    def _get_state(self, name):
        result = self.get_model(name, self.world_frame)
        if not result.success:
            raise RuntimeError("get_model_state({}) failed: {}".format(name, result.status_message))
        return result

    def _remember_model(self, name):
        if name not in self.original_states and name not in self.spawned_models:
            self.original_states[name] = self._get_state(name)
        if name not in self.managed_models:
            self.managed_models.append(name)

    def _set_pose(self, name, pose):
        state = ModelState()
        state.model_name = name
        state.pose = pose
        state.reference_frame = self.world_frame
        result = self.set_model(state)
        if not result.success:
            raise RuntimeError("set_model_state({}) failed: {}".format(name, result.status_message))
        actual = self._get_state(name)
        requested = np.array([pose.position.x, pose.position.y, pose.position.z])
        observed = np.array([actual.pose.position.x, actual.pose.position.y, actual.pose.position.z])
        if np.linalg.norm(requested - observed) > 0.03:
            raise RuntimeError("Gazebo did not place {} at requested pose".format(name))
        return actual.pose

    def _discover(self):
        world = self.get_world()
        if not world.success:
            raise RuntimeError("get_world_properties failed: {}".format(world.status_message))
        names = list(world.model_names)
        self.robot_model = self._select_model(
            names, self.robot_model_param, self.robot_model_regex, "robot"
        )
        self.target_model = self._select_model(
            names, self.target_model_param, self.target_model_regex, "red sphere"
        )
        managed_pattern = re.compile(self.managed_model_regex, re.IGNORECASE)
        for name in names:
            if managed_pattern.search(name):
                self._remember_model(name)
        self._remember_model(self.target_model)
        self.robot_state = self._get_state(self.robot_model)

        with self.lock:
            camera_info = self.latest_camera_info
            hazard_message = self.latest_hazard
        self.camera_frame = self.camera_frame_param or camera_info.header.frame_id
        self.output_frame = self.output_frame_param or hazard_message.header.frame_id
        if not self.camera_frame or not self.output_frame:
            raise RuntimeError("CameraInfo and hazards_3d must provide frame_id")

        self._refresh_transforms()

        self.discovery = {
            "gazebo_world_frame": self.world_frame,
            "gazebo_model_count": len(names),
            "gazebo_models": names,
            "robot_model": self.robot_model,
            "robot_pose_world": self._pose_record(self.robot_state.pose),
            "red_sphere_model": self.target_model,
            "red_sphere_original_pose_world": self._pose_record(self.original_states[self.target_model].pose),
            "camera_frame_from_camera_info": camera_info.header.frame_id,
            "camera_frame_used": self.camera_frame,
            "output_frame_from_hazards": hazard_message.header.frame_id,
            "output_frame_used": self.output_frame,
            "base_frame": self.base_frame,
            "tf_base_from_camera": self._transform_record(self.base_from_camera.transform),
            "tf_base_from_output": self._transform_record(self.base_from_output.transform),
            "derived_camera_pose_world": self._pose_record(_matrix_to_pose(self.world_from_camera)),
            "managed_original_models": sorted(self.original_states),
        }

    def _refresh_transforms(self):
        """Re-read robot truth and TF so a settling robot cannot stale the geometry."""
        self.robot_state = self._get_state(self.robot_model)
        self.base_from_camera = self.tf_buffer.lookup_transform(
            self.base_frame, self.camera_frame, rospy.Time(0), rospy.Duration(self.tf_timeout)
        )
        self.base_from_output = self.tf_buffer.lookup_transform(
            self.base_frame, self.output_frame, rospy.Time(0), rospy.Duration(self.tf_timeout)
        )
        world_from_base = _pose_to_matrix(self.robot_state.pose)
        self.world_from_camera = world_from_base.dot(
            _transform_to_matrix(self.base_from_camera.transform)
        )
        self.world_from_output = world_from_base.dot(
            _transform_to_matrix(self.base_from_output.transform)
        )
        self.output_from_world = np.linalg.inv(self.world_from_output)

    @staticmethod
    def _pose_record(pose):
        return {
            "position": {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z},
            "orientation": {
                "x": pose.orientation.x, "y": pose.orientation.y,
                "z": pose.orientation.z, "w": pose.orientation.w,
            },
        }

    @staticmethod
    def _transform_record(transform):
        return {
            "translation": {
                "x": transform.translation.x, "y": transform.translation.y,
                "z": transform.translation.z,
            },
            "rotation": {
                "x": transform.rotation.x, "y": transform.rotation.y,
                "z": transform.rotation.z, "w": transform.rotation.w,
            },
        }

    def _camera_point_to_world_pose(self, distance, angle_deg, vertical_offset=0.0):
        angle = math.radians(angle_deg)
        horizontal_range = math.sqrt(max(0.0, distance ** 2 - vertical_offset ** 2))
        # ROS optical coordinates: +X right, +Y down, +Z forward.
        point_camera = np.array([
            horizontal_range * math.sin(angle), vertical_offset,
            horizontal_range * math.cos(angle), 1.0
        ])
        point_world = self.world_from_camera.dot(point_camera)
        pose_matrix = self.world_from_camera.copy()
        pose_matrix[:3, 3] = point_world[:3]
        return _matrix_to_pose(pose_matrix)

    def _world_point_to_output(self, pose):
        point = np.array([pose.position.x, pose.position.y, pose.position.z, 1.0])
        return self.output_from_world.dot(point)[:3]

    def _park_pose(self, index):
        point_camera = np.array([float(index) * 0.5, 0.0, -self.park_distance, 1.0])
        point_world = self.world_from_camera.dot(point_camera)
        matrix = self.world_from_camera.copy()
        matrix[:3, 3] = point_world[:3]
        return _matrix_to_pose(matrix)

    @staticmethod
    def _sdf(name, shape, size):
        if shape == "box":
            geometry = "<box><size>{0} {0} {0}</size></box>".format(size)
        elif shape == "cylinder":
            geometry = "<cylinder><radius>{}</radius><length>{}</length></cylinder>".format(
                size * 0.5, size * 1.5
            )
        else:
            raise ValueError("Unsupported interference shape {!r}".format(shape))
        return """<?xml version='1.0'?>
<sdf version='1.6'><model name='{name}'><static>true</static><link name='link'>
<collision name='collision'><geometry>{geometry}</geometry></collision>
<visual name='visual'><geometry>{geometry}</geometry><material>
<ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse><specular>0.1 0.1 0.1 1</specular>
</material></visual></link></model></sdf>""".format(name=name, geometry=geometry)

    def _prepare_interferences(self):
        world_names = list(self.get_world().model_names)
        prepared = []
        for index, item in enumerate(self.interferences):
            if not bool(item.get("enabled", True)):
                continue
            shape = str(item.get("shape", "box")).lower()
            name = str(item.get("model_name", "hazard_validation_red_{}".format(shape)))
            regex = str(item.get("model_regex", ""))
            existing = name if name in world_names else ""
            if not existing and regex:
                matches = sorted(value for value in world_names if re.search(regex, value, re.IGNORECASE))
                existing = matches[0] if matches else ""
            if existing:
                name = existing
                self._remember_model(name)
            else:
                pose = self._park_pose(len(self.managed_models) + index + 1)
                response = self.spawn_model(
                    name, self._sdf(name, shape, float(item.get("size_m", 0.30))),
                    "hazard_validation", pose, self.world_frame,
                )
                if not response.success:
                    raise RuntimeError("spawn_sdf_model({}) failed: {}".format(name, response.status_message))
                self.spawned_models.add(name)
                self.managed_models.append(name)
                world_names.append(name)
            prepared.append({
                "model_name": name,
                "shape": shape,
                "distance_m": float(item.get("distance_m", 2.0)),
                "angle_deg": float(item.get("angle_deg", 0.0)),
            })
        self.prepared_interferences = prepared
        self.discovery["temporary_interference_models"] = sorted(self.spawned_models)
        self.discovery["interference_cases"] = prepared

    def _park_all(self, exclude=()):
        excluded = set(exclude)
        for index, name in enumerate(self.managed_models):
            if name in excluded:
                continue
            self._set_pose(name, self._park_pose(index + 1))

    def _move_smoothly(self, name, final_pose):
        """Keep one production track alive while changing controlled test poses."""
        current = self._get_state(name).pose
        start = np.array([current.position.x, current.position.y, current.position.z])
        end = np.array([final_pose.position.x, final_pose.position.y, final_pose.position.z])
        distance = float(np.linalg.norm(end - start))
        steps = max(1, int(math.ceil(distance / self.transition_step)))
        actual = current
        for step in range(1, steps + 1):
            ratio = float(step) / steps
            position = start + ratio * (end - start)
            intermediate = Pose()
            intermediate.position.x, intermediate.position.y, intermediate.position.z = position.tolist()
            intermediate.orientation = final_pose.orientation
            actual = self._set_pose(name, intermediate)
            if step < steps:
                self._sleep_wall(self.transition_step_time)
        return actual

    def _sleep_wall(self, seconds):
        deadline = time.monotonic() + seconds
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))

    def _current_counts(self):
        with self.lock:
            message = self.latest_hazard
        if message is None:
            return {}
        return {int(item.id): int(item.observation_count) for item in message.hazards}

    def _messages_since(self, start):
        with self.lock:
            return [(stamp, message) for stamp, message in self.hazard_messages if stamp >= start]

    def _detections_since(self, start):
        with self.lock:
            return [
                (stamp, message) for stamp, message in self.detection_messages
                if stamp >= start
            ]

    def _diagnostics_since(self, start):
        with self.lock:
            return [
                (stamp, payload) for stamp, payload in self.diagnostic_messages
                if stamp >= start
            ]

    def _eligible(self, hazard):
        return bool(hazard.confirmed) if self.require_confirmed else True

    def _active_hazards(self, message, baseline):
        return [
            hazard for hazard in message.hazards
            if self._eligible(hazard)
            and int(hazard.observation_count) > baseline.get(int(hazard.id), 0)
        ]

    def _position_samples(self, messages, baseline, truth):
        samples = []
        track_ids = set()
        duplicate_max = 0
        raw_snapshots = []
        for received, message in messages:
            active = self._active_hazards(message, baseline)
            near = []
            for hazard in active:
                position = np.array([hazard.position.x, hazard.position.y, hazard.position.z])
                if np.linalg.norm(position - truth) <= self.duplicate_radius:
                    near.append((hazard, position))
            duplicate_max = max(duplicate_max, len(near))
            if active:
                nearest = min(
                    active,
                    key=lambda item: np.linalg.norm(
                        np.array([item.position.x, item.position.y, item.position.z]) - truth
                    ),
                )
                position = np.array([nearest.position.x, nearest.position.y, nearest.position.z])
                samples.append((received, int(nearest.id), position))
                if np.linalg.norm(position - truth) <= self.duplicate_radius:
                    track_ids.add(int(nearest.id))
            raw_snapshots.append({
                "received_monotonic": received,
                "header_stamp": message.header.stamp.to_sec(),
                "frame_id": message.header.frame_id,
                "fresh_hazards": [self._hazard_record(item) for item in active],
            })
        return samples, track_ids, duplicate_max, raw_snapshots

    @staticmethod
    def _hazard_record(hazard):
        return {
            "id": int(hazard.id),
            "position": {"x": hazard.position.x, "y": hazard.position.y, "z": hazard.position.z},
            "confidence": float(hazard.confidence),
            "observation_count": int(hazard.observation_count),
            "confirmed": bool(hazard.confirmed),
            "localization_source": hazard.localization_source,
        }

    def _wait_until_stable(self, baseline, truth):
        self._sleep_wall(self.settle_time)
        deadline = time.monotonic() + self.stability_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            start = time.monotonic() - self.stability_window
            samples, _ids, _dup, _raw = self._position_samples(
                self._messages_since(start), baseline, truth
            )
            if len(samples) >= self.min_samples:
                std_norm = float(np.linalg.norm(np.std([item[2] for item in samples], axis=0)))
                if std_norm <= self.stability_std_threshold:
                    return True
            time.sleep(0.05)
        return False

    def _save_images(self, case_directory):
        with self.lock:
            rgb = self.latest_rgb
            depth = self.latest_depth
            debug = self.latest_debug
        saved = {}
        try:
            bgr = self.bridge.imgmsg_to_cv2(rgb, desired_encoding="bgr8")
            path = os.path.join(case_directory, "rgb.png")
            cv2.imwrite(path, bgr)
            saved["rgb"] = path
        except (CvBridgeError, cv2.error) as error:
            saved["rgb_error"] = str(error)
        try:
            depth_array = np.asarray(self.bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough"))
            if self.save_npy_depth:
                path = os.path.join(case_directory, "depth.npy")
                np.save(path, depth_array)
                saved["depth_npy"] = path
            if depth.encoding == "16UC1":
                depth_mm = depth_array.astype(np.uint16)
            else:
                finite = np.nan_to_num(depth_array.astype(np.float64), nan=0.0, posinf=0.0, neginf=0.0)
                depth_mm = np.clip(np.rint(finite * 1000.0), 0, 65535).astype(np.uint16)
            path = os.path.join(case_directory, "depth_mm.png")
            cv2.imwrite(path, depth_mm)
            saved["depth_png"] = path
            saved["depth_encoding"] = depth.encoding
        except (CvBridgeError, cv2.error, ValueError) as error:
            saved["depth_error"] = str(error)
        if debug is not None:
            try:
                debug_bgr = self.bridge.imgmsg_to_cv2(debug, desired_encoding="bgr8")
                path = os.path.join(case_directory, "detection.png")
                cv2.imwrite(path, debug_bgr)
                saved["detection"] = path
            except (CvBridgeError, cv2.error) as error:
                saved["detection_error"] = str(error)
        return {key: os.path.relpath(value, self.run_directory) if key.endswith(("rgb", "npy", "png", "detection")) else value for key, value in saved.items()}

    def _run_case(self, case_index, case_type, model_name, shape, distance, angle, expected):
        self._refresh_transforms()
        case_robot_pose = self._pose_record(self.robot_state.pose)
        case_camera_pose = self._pose_record(_matrix_to_pose(self.world_from_camera))
        self._park_all(exclude=(model_name,))
        requested_pose = self._camera_point_to_world_pose(
            distance, angle, self.target_vertical_offset
        )
        if expected and model_name == self.target_model and self.target_track_established:
            actual_pose = self._move_smoothly(model_name, requested_pose)
        else:
            actual_pose = self._set_pose(model_name, requested_pose)
        if expected and model_name == self.target_model:
            self.target_track_established = True
        self._sleep_wall(self.reset_settle_time)
        # Transitional observations are intentionally excluded. From here on,
        # only count increments at the requested final pose are evaluated.
        baseline = self._current_counts()
        truth_output = self._world_point_to_output(actual_pose)
        stable = self._wait_until_stable(baseline, truth_output)
        sample_start = time.monotonic()
        self._sleep_wall(self.sample_duration)
        messages = self._messages_since(sample_start)
        detection_messages = self._detections_since(sample_start)
        diagnostic_messages = self._diagnostics_since(sample_start)
        matched, target_ids, duplicate_max, raw = self._position_samples(
            messages, baseline, truth_output
        )

        two_d_snapshots = []
        for received, message in detection_messages:
            two_d_snapshots.append({
                "received_monotonic": received,
                "header_stamp": message.header.stamp.to_sec(),
                "frame_id": message.header.frame_id,
                "detections": [{
                    "bbox": [item.x, item.y, item.width, item.height],
                    "center": [item.center_x, item.center_y],
                    "area": item.area,
                    "circularity": item.circularity,
                    "aspect_ratio": item.aspect_ratio,
                } for item in message.detections],
            })
        diagnostic_snapshots = [
            {"received_monotonic": received, "payload": payload}
            for received, payload in diagnostic_messages
        ]
        diagnostic_candidates = [
            candidate
            for _received, payload in diagnostic_messages
            for candidate in payload.get("candidates", [])
        ]
        two_d_detected = any(snapshot["detections"] for snapshot in two_d_snapshots)
        three_d_available = any(
            bool(candidate.get("three_d_available", False))
            for candidate in diagnostic_candidates
        )
        sources = [
            str(candidate.get("source", "2d_only"))
            for candidate in diagnostic_candidates
        ]
        source = (
            max(set(sources), key=lambda value: (sources.count(value), value != "2d_only"))
            if sources else "2d_only"
        )

        positions = [item[2] for item in matched]
        metrics = compute_sample_metrics(positions, truth_output)
        fresh_ids = set()
        for snapshot in raw:
            fresh_ids.update(item["id"] for item in snapshot["fresh_hazards"])
        detected = bool(positions)
        success = bool(
            detected and metrics["mean_error_m"] is not None
            and metrics["mean_error_m"] <= self.max_localization_error
        ) if expected else not bool(fresh_ids)
        false_positive_count = 0 if expected else len(fresh_ids)
        duplicate_count = max(0, duplicate_max - 1) if expected else 0
        matched_track_id = None
        if matched:
            matched_track_id = max(
                set(item[1] for item in matched),
                key=lambda track_id: sum(1 for item in matched if item[1] == track_id),
            )

        case_id = "{:03d}_{}_{}m_{:+g}deg".format(
            case_index, case_type, ("{:g}".format(distance)), angle
        ).replace("+", "p").replace("-", "m")
        case_directory = os.path.join(self.run_directory, case_id)
        os.makedirs(case_directory)
        images = self._save_images(case_directory)
        notes = []
        if not stable:
            notes.append("stability criterion not reached before timeout")
        if expected and not detected:
            notes.append("no fresh confirmed detection during sample window")
        if expected and not two_d_detected:
            notes.append("no 2D detection during sample window")
        if expected and two_d_detected and not three_d_available:
            notes.append("2D detected but no 3D source was available")
        if expected and detected and not success:
            notes.append("mean localization error exceeds threshold")
        if not expected and false_positive_count:
            notes.append("fresh confirmed detection caused by red shape distractor")

        record = {
            "case_id": case_id,
            "case_type": case_type,
            "model_name": model_name,
            "shape": shape,
            "distance_m": distance,
            "angle_deg": angle,
            "expected_hazard": expected,
            "two_d_detected": two_d_detected,
            "two_d_sample_count": len(two_d_snapshots),
            "three_d_available": three_d_available,
            "source": source,
            "detected": detected,
            "success": success,
            "stable": stable,
            "baseline_observation_counts": baseline,
            "matched_track_id": matched_track_id,
            "truth_world": self._pose_record(actual_pose),
            "truth_output_frame": _xyz(truth_output),
            "output_frame": self.output_frame,
            "robot_pose_world": case_robot_pose,
            "camera_pose_world": case_camera_pose,
            "unique_target_track_count": len(target_ids),
            "duplicate_target_count": duplicate_count,
            "false_positive_count": false_positive_count,
            "fresh_track_ids": sorted(fresh_ids),
            "images": images,
            "notes": "; ".join(notes),
        }
        record.update(metrics)
        with open(os.path.join(case_directory, "detections.json"), "w", encoding="utf-8") as output:
            json.dump(raw, output, ensure_ascii=False, indent=2, sort_keys=True)
            output.write("\n")
        with open(os.path.join(case_directory, "two_d_detections.json"), "w", encoding="utf-8") as output:
            json.dump(two_d_snapshots, output, ensure_ascii=False, indent=2, sort_keys=True)
            output.write("\n")
        with open(os.path.join(case_directory, "geometry_diagnostics.json"), "w", encoding="utf-8") as output:
            json.dump(diagnostic_snapshots, output, ensure_ascii=False, indent=2, sort_keys=True)
            output.write("\n")
        with open(os.path.join(case_directory, "metrics.json"), "w", encoding="utf-8") as output:
            json.dump(record, output, ensure_ascii=False, indent=2, sort_keys=True)
            output.write("\n")
        rospy.loginfo(
            "[%s] 2d=%s 3d_available=%s success=%s source=%s samples=%d "
            "mean_error=%s std=%s duplicates=%d fp=%d",
            case_id, two_d_detected, three_d_available, success, source,
            metrics["sample_count"], metrics["mean_error_m"], metrics["std_norm_m"],
            duplicate_count, false_positive_count,
        )
        return record

    @staticmethod
    def _csv_row(case):
        mean = case["mean_position"] or {}
        std = case["std_xyz_m"] or {}
        truth = case["truth_output_frame"]
        return {
            "case_id": case["case_id"], "case_type": case["case_type"],
            "model_name": case["model_name"], "shape": case["shape"],
            "distance_m": case["distance_m"], "angle_deg": case["angle_deg"],
            "expected_hazard": case["expected_hazard"],
            "two_d_detected": case["two_d_detected"],
            "three_d_available": case["three_d_available"],
            "detected": case["detected"], "success": case["success"],
            "source": case["source"], "stable": case["stable"],
            "two_d_sample_count": case["two_d_sample_count"],
            "sample_count": case["sample_count"], "matched_track_id": case["matched_track_id"],
            "truth_x": truth["x"], "truth_y": truth["y"], "truth_z": truth["z"],
            "mean_x": mean.get("x"), "mean_y": mean.get("y"), "mean_z": mean.get("z"),
            "mean_error_m": case["mean_error_m"], "rmse_m": case["rmse_m"],
            "max_error_m": case["max_error_m"], "std_x_m": std.get("x"),
            "std_y_m": std.get("y"), "std_z_m": std.get("z"),
            "std_norm_m": case["std_norm_m"],
            "unique_target_track_count": case["unique_target_track_count"],
            "duplicate_target_count": case["duplicate_target_count"],
            "false_positive_count": case["false_positive_count"], "notes": case["notes"],
        }

    def _write_reports(self, cases, status, error=""):
        summary = summarize_cases(cases)
        payload = {
            "schema": "hazard_gazebo_validation_v1",
            "status": status,
            "error": error,
            "created_at": datetime.now().isoformat(),
            "run_directory": self.run_directory,
            "configuration": {
                "distances_m": self.distances, "angles_deg": self.angles,
                "use_livox": False, "require_confirmed": self.require_confirmed,
                "max_localization_error_m": self.max_localization_error,
                "duplicate_radius_m": self.duplicate_radius,
                "settle_time_s": self.settle_time,
                "stability_timeout_s": self.stability_timeout,
                "stability_std_threshold_m": self.stability_std_threshold,
                "sample_duration_s": self.sample_duration,
                "transition_step_m": self.transition_step,
                "transition_step_time_s": self.transition_step_time,
            },
            "discovery": getattr(self, "discovery", {}),
            "summary": summary,
            "cases": cases,
        }
        json_path = os.path.join(self.run_directory, "summary.json")
        with open(json_path, "w", encoding="utf-8") as output:
            json.dump(payload, output, ensure_ascii=False, indent=2, sort_keys=True)
            output.write("\n")
        csv_path = os.path.join(self.run_directory, "summary.csv")
        with open(csv_path, "w", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=CSV_FIELDS)
            writer.writeheader()
            writer.writerows(self._csv_row(case) for case in cases)
        return summary, json_path, csv_path

    def restore_scene(self):
        if self.restored:
            return
        self.restored = True
        for name, state in self.original_states.items():
            try:
                self._set_pose(name, state.pose)
                rospy.loginfo("Restored Gazebo model %s", name)
            except Exception as error:  # best effort during ROS shutdown
                rospy.logerr("Failed to restore %s: %s", name, error)
        for name in sorted(self.spawned_models):
            try:
                result = self.delete_model(name)
                if not result.success:
                    rospy.logerr("Failed to delete temporary model %s: %s", name, result.status_message)
                else:
                    rospy.loginfo("Deleted temporary Gazebo model %s", name)
            except Exception as error:
                rospy.logerr("Failed to delete temporary model %s: %s", name, error)

    def run(self):
        cases = []
        status = "failed"
        error_message = ""
        try:
            self._wait_for_inputs()
            self._discover()
            self._prepare_interferences()
            self._park_all()
            self._sleep_wall(self.reset_settle_time)
            with open(os.path.join(self.run_directory, "discovery.json"), "w", encoding="utf-8") as output:
                json.dump(self.discovery, output, ensure_ascii=False, indent=2, sort_keys=True)
                output.write("\n")

            index = 1
            for distance in self.distances:
                for angle in self.angles:
                    if rospy.is_shutdown():
                        raise RuntimeError("validation interrupted")
                    cases.append(self._run_case(
                        index, "hazard", self.target_model, "sphere", distance, angle, True
                    ))
                    index += 1
            for item in self.prepared_interferences:
                if rospy.is_shutdown():
                    raise RuntimeError("validation interrupted")
                cases.append(self._run_case(
                    index, "interference", item["model_name"], item["shape"],
                    item["distance_m"], item["angle_deg"], False,
                ))
                index += 1
            status = "completed"
        except Exception as error:
            error_message = "{}: {}".format(type(error).__name__, error)
            rospy.logerr("Hazard validation failed: %s", error_message)
        finally:
            self.restore_scene()
            summary, json_path, csv_path = self._write_reports(cases, status, error_message)
            rospy.loginfo("Validation status=%s summary=%s", status, summary)
            rospy.loginfo("Reports: %s %s", json_path, csv_path)
        if status != "completed":
            raise RuntimeError(error_message)


def main():
    rospy.init_node("hazard_gazebo_validation")
    try:
        node = HazardValidationNode()
        node.run()
    except Exception as error:
        rospy.logfatal("Gazebo hazard validation terminated: %s", error)
        raise SystemExit(2)


if __name__ == "__main__":
    main()
