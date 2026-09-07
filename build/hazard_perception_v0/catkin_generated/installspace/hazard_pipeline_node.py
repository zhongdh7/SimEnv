#!/usr/bin/env python3
"""Complete red-hazard RGB-D, tracking and optional Livox ROS pipeline."""

import collections
import json
import os
import threading

import cv2
import message_filters
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as point_cloud2
import tf2_geometry_msgs  # noqa: F401 - registers geometry conversions with tf2
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PointStamped, TransformStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import quaternion_matrix
from visualization_msgs.msg import Marker, MarkerArray

from hazard_perception_v0.detector import DetectorConfig, RedSphereDetector
from hazard_perception_v0.livox_fusion import choose_fused_position, estimate_livox_position
from hazard_perception_v0.localization import (
    CameraIntrinsics,
    backproject_pixel,
    camera_models_aligned,
    depth_to_metres,
    fit_sphere_robust,
    masked_depth_points,
    monocular_sphere_position,
    plane_fit_residual,
    radial_depth_correlation,
    rasterize_point_cloud_depth,
    robust_depth_from_detection,
    optical_to_sensor_body,
    validate_sphere_geometry,
)
from hazard_perception_v0.msg import (
    Hazard3D,
    Hazard3DArray,
    HazardDetection2D,
    HazardDetection2DArray,
)
from hazard_perception_v0.result_manager import ResultManager
from hazard_perception_v0.tracker import HazardObservation, HazardTracker


class HazardPipelineNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.detector = RedSphereDetector(self._load_detector_config())
        self._load_parameters()
        self.tracker = HazardTracker(
            confirmation_count=self.confirmation_count,
            merge_distance=self.merge_distance,
            smoothing_alpha=self.smoothing_alpha,
            candidate_timeout=self.candidate_timeout,
            max_position_jump=self.max_position_jump,
            final_merge_distance=self.final_merge_distance,
        )
        self.result_manager = ResultManager(
            self.output_directory,
            save_csv=self.save_csv,
            run_id=self.run_id,
            frame_id=self.start_frame,
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.start_frame_ready = False
        self._finalize_paths = None
        self._depth_logged = False
        self._alignment_result = None
        self._track_diagnostics = {}
        self._diagnostics_lock = threading.Lock()
        self._cache_lock = threading.Lock()
        self._depth_clouds = collections.deque(maxlen=self.cloud_cache_size)
        self._livox_clouds = collections.deque(maxlen=self.cloud_cache_size)

        self.hazard_publisher = rospy.Publisher(
            self.hazards_topic, Hazard3DArray, queue_size=1, latch=True
        )
        self.detections_publisher = rospy.Publisher(
            self.detections_topic, HazardDetection2DArray, queue_size=1
        )
        self.geometry_diagnostics_publisher = rospy.Publisher(
            self.geometry_diagnostics_topic, String, queue_size=1
        )
        self.marker_publisher = rospy.Publisher(
            self.markers_topic, MarkerArray, queue_size=1, latch=True
        )
        self.mask_publisher = rospy.Publisher(self.pipeline_mask_topic, Image, queue_size=1)
        self.debug_publisher = rospy.Publisher(
            self.pipeline_debug_topic, Image, queue_size=1
        )

        self.depth_cloud_subscriber = rospy.Subscriber(
            self.depth_points_topic,
            PointCloud2,
            self._depth_cloud_callback,
            queue_size=2,
            buff_size=2**25,
        )
        self.livox_subscriber = None
        if self.use_livox:
            self._subscribe_livox()

        self.rgb_subscriber = message_filters.Subscriber(
            self.rgb_topic, Image, queue_size=1, buff_size=2**24
        )
        self.depth_subscriber = message_filters.Subscriber(
            self.depth_topic, Image, queue_size=1, buff_size=2**24
        )
        self.rgb_info_subscriber = message_filters.Subscriber(
            self.rgb_info_topic, CameraInfo, queue_size=1
        )
        self.depth_info_subscriber = message_filters.Subscriber(
            self.depth_info_topic, CameraInfo, queue_size=1
        )
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [
                self.rgb_subscriber,
                self.depth_subscriber,
                self.rgb_info_subscriber,
                self.depth_info_subscriber,
            ],
            queue_size=self.sync_queue_size,
            slop=self.sync_tolerance,
            allow_headerless=False,
        )
        self.synchronizer.registerCallback(self._synchronized_callback)

        self.finalize_service = rospy.Service(
            self.finalize_service_name, Trigger, self._finalize_callback
        )
        self.start_timer = rospy.Timer(rospy.Duration(0.25), self._initialize_start_frame)
        self.publish_timer = rospy.Timer(rospy.Duration(1.0), self._timer_callback)
        rospy.loginfo(
            "hazard_pipeline syncing RGB=%s depth=%s RGB-info=%s depth-info=%s "
            "queue=%d slop=%.3fs use_livox=%s",
            self.rgb_topic,
            self.depth_topic,
            self.rgb_info_topic,
            self.depth_info_topic,
            self.sync_queue_size,
            self.sync_tolerance,
            self.use_livox,
        )

    @staticmethod
    def _load_detector_config():
        defaults = DetectorConfig()
        values = {
            name: rospy.get_param("~{}".format(name), getattr(defaults, name))
            for name in defaults.__dataclass_fields__
        }
        return DetectorConfig(**values)

    def _load_parameters(self):
        get = rospy.get_param
        self.rgb_topic = get("~rgb_topic", "/real_sense/rgb/image_raw")
        self.camera_frame = get("~camera_frame", "real_sense")
        self.depth_topic = get("~depth_topic", "/real_sense/depth/image_raw")
        self.rgb_info_topic = get("~rgb_info_topic", "/real_sense/rgb/camera_info")
        self.depth_info_topic = get("~depth_info_topic", "/real_sense/depth/camera_info")
        self.depth_points_topic = get("~depth_points_topic", "/real_sense/depth/points")
        self.livox_topic = get("~livox_topic", "/scan")
        self.livox_message_type = str(get("~livox_message_type", "auto")).lower()
        self.hazards_topic = get("~hazards_topic", "/hazard_perception_v0/hazards_3d")
        self.detections_topic = get("~detections_topic", "/hazard_perception_v0/detections")
        self.geometry_diagnostics_topic = get(
            "~geometry_diagnostics_topic", "/hazard_perception_v0/geometry_diagnostics"
        )
        self.markers_topic = get("~markers_topic", "/hazard_perception_v0/markers")
        self.pipeline_mask_topic = get(
            "~pipeline_mask_topic", "/hazard_perception_v0/pipeline_red_mask"
        )
        self.pipeline_debug_topic = get(
            "~pipeline_debug_topic", "/hazard_perception_v0/pipeline_debug_image"
        )
        self.finalize_service_name = get("~finalize_service", "/hazard_perception_v0/finalize")

        self.sync_queue_size = int(get("~sync_queue_size", 10))
        self.sync_tolerance = float(get("~sync_tolerance", 0.08))
        self.cloud_cache_size = int(get("~cloud_cache_size", 10))
        self.depth_cloud_tolerance = float(get("~depth_cloud_tolerance", 0.12))
        self.alignment_mode = str(get("~alignment_mode", "auto")).lower()
        if self.alignment_mode not in ("auto", "aligned", "pointcloud"):
            raise ValueError("alignment_mode must be auto, aligned or pointcloud")

        self.min_depth = float(get("~min_depth", 0.4))
        self.max_depth = float(get("~max_depth", 8.0))
        self.min_depth_pixels = int(get("~min_depth_pixels", 20))
        self.depth_mad_scale = float(get("~depth_mad_scale", 3.5))
        self.camera_model_tolerance = float(get("~camera_model_tolerance", 1.0e-3))

        self.sphere_fit_enabled = bool(get("~sphere_fit_enabled", True))
        self.sphere_fit_mode = str(get("~sphere_fit_mode", "known_radius")).lower()
        self.sphere_radius = float(get("~sphere_radius_m", 0.15))
        self.sphere_min_radius = float(get("~sphere_min_radius_m", 0.10))
        self.sphere_max_radius = float(get("~sphere_max_radius_m", 0.22))
        self.sphere_fit_min_points = int(get("~sphere_fit_min_points", 80))
        self.edge_sphere_fit_min_points = int(get("~edge_sphere_fit_min_points", 35))
        self.sphere_fit_max_points = int(get("~sphere_fit_max_points", 4000))
        self.sphere_fit_max_iterations = int(get("~sphere_fit_max_iterations", 20))
        self.sphere_fit_huber_delta = float(get("~sphere_fit_huber_delta_m", 0.008))
        self.sphere_fit_max_residual = float(get("~sphere_fit_max_residual_m", 0.012))
        self.edge_sphere_fit_max_residual = float(
            get("~edge_sphere_fit_max_residual_m", 0.018)
        )
        self.sphere_plane_ratio_max = float(get("~sphere_plane_ratio_max", 0.65))
        self.sphere_min_plane_residual = float(
            get("~sphere_min_plane_residual_m", 0.003)
        )
        self.sphere_min_radial_correlation = float(
            get("~sphere_min_radial_correlation", 0.20)
        )
        self.edge_min_radial_correlation = float(
            get("~edge_min_radial_correlation", 0.05)
        )
        self.sphere_min_inlier_ratio = float(get("~sphere_min_inlier_ratio", 0.65))
        self.geometry_fallback_confidence_scale = float(
            get("~geometry_fallback_confidence_scale", 0.40)
        )
        self.edge_confirmation_count = int(get("~edge_confirmation_count", 6))
        self.enable_monocular_fallback = bool(get("~enable_monocular_fallback", False))
        self.monocular_confidence_scale = float(get("~monocular_confidence_scale", 0.25))
        self.livox_sphere_fit_min_points = int(get("~livox_sphere_fit_min_points", 12))
        self.livox_sphere_fit_max_residual = float(
            get("~livox_sphere_fit_max_residual_m", 0.025)
        )
        self.livox_sphere_plane_ratio_max = float(
            get("~livox_sphere_plane_ratio_max", 0.80)
        )
        self.livox_sphere_min_plane_residual = float(
            get("~livox_sphere_min_plane_residual_m", 0.001)
        )
        self.livox_sphere_min_inlier_ratio = float(
            get("~livox_sphere_min_inlier_ratio", 0.50)
        )

        if self.sphere_fit_mode not in ("known_radius", "free_radius"):
            raise ValueError("sphere_fit_mode must be known_radius or free_radius")
        if self.sphere_radius <= 0.0 or self.sphere_fit_min_points < 4:
            raise ValueError("Invalid sphere radius or minimum point count")

        self.odom_frame = get("~odom_frame", "odom")
        self.base_frame = get("~base_frame", "base")
        self.start_frame = get("~start_frame", "start_frame")
        self.tf_timeout = float(get("~tf_timeout", 0.15))
        self.confirmation_count = int(get("~confirmation_count", 3))
        if self.edge_confirmation_count < self.confirmation_count:
            raise ValueError("edge_confirmation_count cannot be below confirmation_count")
        self.merge_distance = float(get("~merge_distance", 0.6))
        self.smoothing_alpha = float(get("~smoothing_alpha", 0.35))
        self.candidate_timeout = float(get("~candidate_timeout", 3.0))
        self.max_position_jump = float(get("~max_position_jump", 0.45))
        self.final_merge_distance = float(get("~final_merge_distance", 0.6))

        self.use_livox = bool(get("~use_livox", False))
        self.livox_min_points = int(get("~livox_min_points", 3))
        self.livox_sync_tolerance = float(get("~livox_sync_tolerance", 0.15))
        self.livox_min_range = float(get("~livox_min_range", 0.2))
        self.livox_max_range = float(get("~livox_max_range", 40.0))
        self.livox_cluster_tolerance = float(get("~livox_cluster_tolerance", 0.25))
        self.fusion_max_disagreement = float(get("~fusion_max_disagreement", 0.5))
        self.rgbd_weight = float(get("~rgbd_weight", 0.7))
        self.livox_weight = float(get("~livox_weight", 0.3))

        self.publish_candidates = bool(get("~publish_candidates", True))
        self.publish_debug_images = bool(get("~publish_debug_images", True))
        self.output_directory = get("~output_directory", "results")
        self.save_csv = bool(get("~save_csv", False))
        self.run_id = str(get("~run_id", ""))
        self.diagnostics_log_path = os.path.expanduser(
            get("~diagnostics_log_path", "")
        )
        if not self.diagnostics_log_path and self.run_id:
            self.diagnostics_log_path = os.path.join(
                os.path.expanduser(self.output_directory),
                "hazard_diagnostics_%s.jsonl" % self.run_id,
            )

    def _subscribe_livox(self):
        message_type = self.livox_message_type
        if message_type == "auto":
            published = dict(rospy.get_published_topics())
            ros_type = published.get(self.livox_topic, "sensor_msgs/PointCloud")
            message_type = "pointcloud2" if ros_type == "sensor_msgs/PointCloud2" else "pointcloud"
        if message_type == "pointcloud2":
            message_class = PointCloud2
        elif message_type == "pointcloud":
            message_class = PointCloud
        else:
            raise ValueError("livox_message_type must be auto, pointcloud or pointcloud2")
        self.livox_subscriber = rospy.Subscriber(
            self.livox_topic,
            message_class,
            self._livox_cloud_callback,
            queue_size=2,
            buff_size=2**25,
        )
        rospy.loginfo("Livox input %s uses %s", self.livox_topic, message_class._type)

    def _initialize_start_frame(self, _event):
        if self.start_frame_ready:
            return
        try:
            initial_pose = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(self.tf_timeout),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
            rospy.logwarn_throttle(
                3.0, "Waiting for TF %s -> %s: %s", self.odom_frame, self.base_frame, error
            )
            return
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.start_frame
        transform.transform = initial_pose.transform
        self.static_broadcaster.sendTransform(transform)
        self.result_manager.set_start_frame_transform(transform)
        self.start_frame_ready = True
        self.start_timer.shutdown()
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rospy.loginfo(
            "Captured start frame %s in %s: xyz=(%.3f, %.3f, %.3f) "
            "quaternion=(%.4f, %.4f, %.4f, %.4f)",
            self.start_frame, self.odom_frame,
            translation.x, translation.y, translation.z,
            rotation.x, rotation.y, rotation.z, rotation.w,
        )

    def _depth_cloud_callback(self, message):
        with self._cache_lock:
            self._depth_clouds.append(message)

    def _livox_cloud_callback(self, message):
        with self._cache_lock:
            self._livox_clouds.append(message)

    @staticmethod
    def _nearest_message(cache, stamp, tolerance):
        with_stamp = [message for message in cache if not message.header.stamp.is_zero()]
        if not with_stamp:
            return None
        message = min(with_stamp, key=lambda item: abs((item.header.stamp - stamp).to_sec()))
        return message if abs((message.header.stamp - stamp).to_sec()) <= tolerance else None

    @staticmethod
    def _cloud_to_array(message):
        if isinstance(message, PointCloud2):
            values = point_cloud2.read_points(
                message, field_names=("x", "y", "z"), skip_nans=True
            )
            points = np.asarray(list(values), dtype=np.float64)
        else:
            points = np.asarray([[p.x, p.y, p.z] for p in message.points], dtype=np.float64)
        return points.reshape((-1, 3)) if points.size else np.empty((0, 3), dtype=np.float64)

    def _transform_points(self, points, source_frame, target_frame, stamp):
        if source_frame == target_frame:
            return points
        transform = self.tf_buffer.lookup_transform(
            target_frame, source_frame, stamp, rospy.Duration(self.tf_timeout)
        )
        rotation = transform.transform.rotation
        matrix = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])[:3, :3]
        translation = transform.transform.translation
        offset = np.asarray([translation.x, translation.y, translation.z], dtype=np.float64)
        return points.dot(matrix.T) + offset

    def _registered_depth(self, rgb_stamp, rgb_frame, intrinsics):
        with self._cache_lock:
            cloud = self._nearest_message(
                list(self._depth_clouds), rgb_stamp, self.depth_cloud_tolerance
            )
        if cloud is None:
            rospy.logwarn_throttle(2.0, "Unaligned RGB/depth and no synchronized depth cloud")
            return None
        try:
            points = self._cloud_to_array(cloud)
            points = self._transform_points(
                points, cloud.header.frame_id, rgb_frame, cloud.header.stamp
            )
            return rasterize_point_cloud_depth(points, intrinsics, self.min_depth, self.max_depth)
        except (tf2_ros.TransformException, ValueError) as error:
            rospy.logwarn_throttle(2.0, "Cannot register depth point cloud: %s", error)
            return None

    def _livox_points_in_camera(self, rgb_stamp, rgb_frame):
        if not self.use_livox:
            return None
        with self._cache_lock:
            cloud = self._nearest_message(
                list(self._livox_clouds), rgb_stamp, self.livox_sync_tolerance
            )
        if cloud is None:
            rospy.logwarn_throttle(3.0, "No Livox cloud within %.3fs", self.livox_sync_tolerance)
            return None
        try:
            points = self._cloud_to_array(cloud)
            return self._transform_points(
                points, cloud.header.frame_id, rgb_frame, cloud.header.stamp
            )
        except (tf2_ros.TransformException, ValueError) as error:
            rospy.logwarn_throttle(2.0, "Cannot transform Livox cloud: %s", error)
            return None

    def _frame_audit_record(self, stamp, camera_frame):
        """Return real sensor pose/intrinsics metadata for offline audits.

        This is deliberately diagnostic-only.  It contains TF and CameraInfo
        metadata observed by the running pipeline; it never reads simulator
        truth and is not used by navigation or detection decisions.
        """
        record = {"odom_frame": self.odom_frame, "camera_frame": camera_frame}
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame, camera_frame, stamp,
                rospy.Duration(self.tf_timeout),
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            record["T_odom_camera"] = {
                "parent_frame": transform.header.frame_id,
                "child_frame": transform.child_frame_id,
                "translation_xyz": [float(translation.x), float(translation.y), float(translation.z)],
                "quaternion_xyzw": [float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)],
            }
        except tf2_ros.TransformException as error:
            record["T_odom_camera_error"] = str(error)
        return record

    def _alignment_is_valid(self, rgb_message, depth_message, rgb_info, depth_info):
        rgb_model = CameraIntrinsics.from_camera_info(rgb_info)
        depth_model = CameraIntrinsics.from_camera_info(depth_info)
        aligned = bool(
            rgb_message.width == depth_message.width
            and rgb_message.height == depth_message.height
            and rgb_message.header.frame_id == depth_message.header.frame_id
            and rgb_info.header.frame_id == depth_info.header.frame_id
            and camera_models_aligned(
                rgb_model, depth_model, absolute_tolerance=self.camera_model_tolerance
            )
        )
        if self.alignment_mode == "aligned":
            return True
        if self.alignment_mode == "pointcloud":
            return False
        return aligned

    def _log_depth_characteristics(self, depth, encoding):
        if self._depth_logged:
            return
        with np.errstate(invalid="ignore"):
            valid = depth[np.isfinite(depth) & (depth > 0.0)]
        invalid_count = int(depth.size - valid.size)
        if valid.size:
            rospy.loginfo(
                "Depth sample encoding=%s unit=metres valid_min=%.3f valid_max=%.3f "
                "valid=%d invalid(0/NaN/Inf)=%d",
                encoding, float(np.min(valid)), float(np.max(valid)), valid.size, invalid_count,
            )
        else:
            rospy.logwarn("Depth sample encoding=%s contains no positive finite values", encoding)
        self._depth_logged = True

    def _detection_confidence(self, detection):
        if detection.touches_border:
            residual_limit = self.detector.config.edge_max_arc_residual
            arc_score = max(0.0, 1.0 - detection.edge_fit_residual / residual_limit)
            return float(np.clip(0.45 + 0.35 * arc_score, 0.0, 0.82))
        shape = max(0.0, 1.0 - abs(float(detection.aspect_ratio) - 1.0))
        return float(np.clip(0.6 * detection.circularity + 0.4 * shape, 0.0, 1.0))

    @staticmethod
    def _to_detection_message(detection):
        message = HazardDetection2D()
        message.x = detection.x
        message.y = detection.y
        message.width = detection.width
        message.height = detection.height
        message.center_x = detection.center_x
        message.center_y = detection.center_y
        message.area = detection.area
        message.circularity = detection.circularity
        message.aspect_ratio = detection.aspect_ratio
        return message

    def _publish_detections(self, rgb_message, detections):
        array = HazardDetection2DArray()
        array.header = rgb_message.header
        array.image_width = rgb_message.width
        array.image_height = rgb_message.height
        array.detections = [self._to_detection_message(item) for item in detections]
        self.detections_publisher.publish(array)

    def _sphere_geometry(self, depth, mask, bbox, detection, intrinsics):
        diagnostics = {
            "geometry_status": "unavailable",
            "geometry_reason": "sphere_fit_disabled",
            "sphere_fit_mode": self.sphere_fit_mode,
            "sphere_fit_residual_m": None,
            "sphere_fit_radius_m": None,
            "sphere_fit_inliers": 0,
            "sphere_fit_points": 0,
            "plane_fit_residual_m": None,
            "radial_depth_correlation": None,
            "localization_confidence": 0.0,
            "depth_valid_pixels": 0,
        }
        if not self.sphere_fit_enabled or depth is None:
            diagnostics["geometry_reason"] = (
                "sphere_fit_disabled" if not self.sphere_fit_enabled else "no_depth"
            )
            return None, diagnostics
        samples = masked_depth_points(
            depth, mask, bbox, intrinsics, self.min_depth, self.max_depth,
            self.sphere_fit_max_points,
        )
        diagnostics["depth_valid_pixels"] = samples.raw_valid_count
        diagnostics["sphere_fit_points"] = int(samples.points.shape[0])
        required_points = (
            self.edge_sphere_fit_min_points
            if detection.touches_border else self.sphere_fit_min_points
        )
        if samples.points.shape[0] < required_points:
            diagnostics["geometry_reason"] = "insufficient_points"
            return None, diagnostics
        fit = fit_sphere_robust(
            samples.points,
            mode=self.sphere_fit_mode,
            known_radius=self.sphere_radius,
            min_radius=self.sphere_min_radius,
            max_radius=self.sphere_max_radius,
            max_iterations=self.sphere_fit_max_iterations,
            huber_delta=self.sphere_fit_huber_delta,
        )
        if fit is None:
            diagnostics["geometry_reason"] = "fit_failed"
            return None, diagnostics
        plane_residual = plane_fit_residual(samples.points)
        correlation = radial_depth_correlation(
            samples.pixels, samples.depths, detection.center_x, detection.center_y
        )
        validation = validate_sphere_geometry(
            fit,
            plane_residual,
            correlation,
            self.edge_sphere_fit_max_residual
            if detection.touches_border else self.sphere_fit_max_residual,
            self.sphere_plane_ratio_max,
            self.sphere_min_plane_residual,
            self.edge_min_radial_correlation
            if detection.touches_border else self.sphere_min_radial_correlation,
            self.sphere_min_inlier_ratio,
        )
        diagnostics.update({
            "geometry_status": "passed" if validation.accepted else "rejected",
            "geometry_reason": validation.reason,
            "sphere_fit_converged": bool(fit.converged),
            "sphere_fit_residual_m": fit.residual_rms,
            "sphere_fit_radius_m": fit.radius,
            "sphere_fit_inliers": fit.inlier_count,
            "sphere_fit_points": fit.point_count,
            "plane_fit_residual_m": plane_residual,
            "radial_depth_correlation": correlation,
            "localization_confidence": validation.confidence,
        })
        return validation, diagnostics

    def _monocular_position(self, detection, intrinsics):
        radius_pixels = detection.fitted_radius_px
        if radius_pixels <= 0.0:
            radius_pixels = 0.25 * (detection.width + detection.height)
        return monocular_sphere_position(
            detection.center_x,
            detection.center_y,
            radius_pixels,
            self.sphere_radius,
            intrinsics,
        )

    def _point_to_start_frame(self, position, frame_id, stamp):
        # SimEnv's RGB-D plugin labels the image frame ``real_sense`` while
        # CameraInfo/image pixels use ROS optical axes (z forward, x right,
        # y down).  The TF frame itself is the sensor body frame.  Convert the
        # fitted optical point to that body frame before applying the normal
        # start-frame rigid transform; otherwise a valid sphere is published
        # several metres away even though its image/depth geometry is sound.
        point_xyz = np.asarray(position, dtype=np.float64)
        if (str(frame_id) == str(self.camera_frame) and
                not str(frame_id).endswith("optical_frame")):
            point_xyz = optical_to_sensor_body(point_xyz)
        point = PointStamped()
        point.header.stamp = stamp
        point.header.frame_id = frame_id
        point.point = Point(x=float(point_xyz[0]), y=float(point_xyz[1]),
                            z=float(point_xyz[2]))
        transformed = self.tf_buffer.transform(
            point, self.start_frame, rospy.Duration(self.tf_timeout)
        )
        return np.asarray(
            [transformed.point.x, transformed.point.y, transformed.point.z], dtype=np.float64
        )

    def _synchronized_callback(self, rgb_message, depth_message, rgb_info, depth_info):
        if not self.start_frame_ready:
            rospy.logwarn_throttle(2.0, "Skipping synchronized input until start_frame is ready")
            return
        try:
            bgr = self.bridge.imgmsg_to_cv2(rgb_message, desired_encoding="bgr8")
            raw_depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
            depth = depth_to_metres(np.asarray(raw_depth), depth_message.encoding)
            mask, detections = self.detector.detect(bgr)
            intrinsics = CameraIntrinsics.from_camera_info(rgb_info)
            intrinsics.validate()
        except (CvBridgeError, ValueError) as error:
            rospy.logerr_throttle(2.0, "Cannot process synchronized RGB-D input: %s", error)
            return

        self._publish_detections(rgb_message, detections)

        self._log_depth_characteristics(depth, depth_message.encoding)
        aligned = self._alignment_is_valid(
            rgb_message, depth_message, rgb_info, depth_info
        )
        if aligned != self._alignment_result:
            self._alignment_result = aligned
            rospy.loginfo(
                "RGB/depth alignment decision: %s",
                "direct registered pixels" if aligned else "depth point-cloud projection",
            )
        sampling_depth = depth if aligned else self._registered_depth(
            rgb_message.header.stamp, rgb_message.header.frame_id, intrinsics
        )
        livox_points = self._livox_points_in_camera(
            rgb_message.header.stamp, rgb_message.header.frame_id
        )
        annotations = []
        frame_diagnostics = []

        for detection in detections:
            bbox = (detection.x, detection.y, detection.width, detection.height)
            rgbd_position = None
            depth_estimate = None
            geometry_validation = None
            geometry_diagnostics = {
                "geometry_status": "unavailable",
                "geometry_reason": "no_depth",
                "localization_confidence": 0.0,
                "depth_valid_pixels": 0,
            }
            if sampling_depth is not None:
                try:
                    depth_estimate = robust_depth_from_detection(
                        sampling_depth,
                        mask,
                        bbox,
                        self.min_depth,
                        self.max_depth,
                        self.min_depth_pixels,
                        self.depth_mad_scale,
                    )
                    geometry_validation, geometry_diagnostics = self._sphere_geometry(
                        sampling_depth, mask, bbox, detection, intrinsics
                    )
                    if geometry_validation is not None and geometry_validation.accepted:
                        rgbd_position = geometry_validation.sphere.center
                        geometry_diagnostics["rgbd_method"] = "sphere_fit"
                    elif geometry_validation is None and depth_estimate is not None:
                        # Preserve the legacy estimate as an unconfirmed fallback candidate.
                        rgbd_position = backproject_pixel(
                            depth_estimate.pixel_u, depth_estimate.pixel_v,
                            depth_estimate.depth, intrinsics,
                        )
                        geometry_diagnostics["rgbd_method"] = "median_fallback"
                except ValueError as error:
                    rospy.logwarn_throttle(2.0, "RGB-D localization failed: %s", error)
                    geometry_diagnostics["geometry_reason"] = "exception:{}".format(error)

            livox_position = None
            livox_count = 0
            livox_geometry_validation = None
            livox_geometry_diagnostics = {
                "livox_geometry_status": "unavailable",
                "livox_sphere_fit_residual_m": None,
                "livox_sphere_fit_radius_m": None,
            }
            if livox_points is not None:
                try:
                    estimate = estimate_livox_position(
                        livox_points,
                        mask,
                        bbox,
                        intrinsics,
                        self.livox_min_range,
                        self.livox_max_range,
                        self.livox_min_points,
                        self.livox_cluster_tolerance,
                    )
                    if estimate is not None:
                        livox_position = estimate.position
                        livox_count = estimate.point_count
                        if estimate.point_count >= self.livox_sphere_fit_min_points:
                            livox_fit = fit_sphere_robust(
                                estimate.points,
                                mode=self.sphere_fit_mode,
                                known_radius=self.sphere_radius,
                                min_radius=self.sphere_min_radius,
                                max_radius=self.sphere_max_radius,
                                max_iterations=self.sphere_fit_max_iterations,
                                huber_delta=self.sphere_fit_huber_delta,
                            )
                            if livox_fit is not None:
                                livox_geometry_validation = validate_sphere_geometry(
                                    livox_fit,
                                    plane_fit_residual(estimate.points),
                                    1.0,
                                    self.livox_sphere_fit_max_residual,
                                    self.livox_sphere_plane_ratio_max,
                                    self.livox_sphere_min_plane_residual,
                                    -1.0,
                                    self.livox_sphere_min_inlier_ratio,
                                )
                                livox_geometry_diagnostics.update({
                                    "livox_geometry_status": (
                                        "passed" if livox_geometry_validation.accepted
                                        else "rejected"
                                    ),
                                    "livox_geometry_reason": livox_geometry_validation.reason,
                                    "livox_sphere_fit_residual_m": livox_fit.residual_rms,
                                    "livox_sphere_fit_radius_m": livox_fit.radius,
                                    "livox_plane_fit_residual_m": (
                                        livox_geometry_validation.plane_residual_rms
                                    ),
                                })
                                if livox_geometry_validation.accepted:
                                    livox_position = livox_fit.center
                                else:
                                    # A rejected Livox fit must not perturb an
                                    # otherwise valid RGB-D sphere centre.
                                    livox_position = None
                except ValueError as error:
                    rospy.logwarn_throttle(2.0, "Livox localization failed: %s", error)

            candidate_diagnostics = {
                "bbox": list(bbox),
                "center": [detection.center_x, detection.center_y],
                "touches_border": bool(detection.touches_border),
                "edge_fit_residual": (
                    detection.edge_fit_residual if detection.touches_border else None
                ),
                "two_d_confidence": self._detection_confidence(detection),
                "two_d_detected": True,
                "livox_point_count": livox_count,
            }
            candidate_diagnostics.update(geometry_diagnostics)
            candidate_diagnostics.update(livox_geometry_diagnostics)

            # A solved but rejected fit is positive evidence of non-spherical
            # structure. It must never enter the production confirmation list.
            if geometry_validation is not None and not geometry_validation.accepted:
                candidate_diagnostics.update({
                    "source": "2d_only",
                    "candidate_state": "geometry_rejected",
                    "three_d_available": False,
                    "track_id": None,
                    "confirmed": False,
                })
                frame_diagnostics.append(candidate_diagnostics)
                annotations.append(
                    "geometry reject {} sr={:.3f} pr={:.3f}".format(
                        geometry_validation.reason,
                        geometry_validation.sphere.residual_rms,
                        geometry_validation.plane_residual_rms,
                    )
                )
                continue
            if (
                geometry_validation is None
                and livox_geometry_validation is not None
                and not livox_geometry_validation.accepted
            ):
                candidate_diagnostics.update({
                    "source": "2d_only",
                    "candidate_state": "livox_geometry_rejected",
                    "three_d_available": False,
                    "track_id": None,
                    "confirmed": False,
                })
                frame_diagnostics.append(candidate_diagnostics)
                annotations.append(
                    "Livox geometry reject {} lr={:.3f}".format(
                        livox_geometry_validation.reason,
                        livox_geometry_validation.sphere.residual_rms,
                    )
                )
                continue

            try:
                result = choose_fused_position(
                    rgbd_position,
                    livox_position,
                    self.fusion_max_disagreement,
                    self.rgbd_weight,
                    self.livox_weight,
                )
            except ValueError as error:
                rospy.logerr_throttle(2.0, "Invalid fusion parameters: %s", error)
                result = None
            result_position = result.position if result is not None else None
            result_source = result.source if result is not None else "2d_only"
            if result_position is None and self.enable_monocular_fallback:
                result_position = self._monocular_position(detection, intrinsics)
                if result_position is not None:
                    result_source = "monocular"
            if result is None:
                if result_position is None:
                    candidate_diagnostics.update({
                        "source": "2d_only",
                        "candidate_state": "waiting_for_depth_or_livox",
                        "three_d_available": False,
                        "track_id": None,
                        "confirmed": False,
                    })
                    frame_diagnostics.append(candidate_diagnostics)
                    annotations.append("2D only: waiting for depth/Livox")
                    continue
            try:
                position_start = self._point_to_start_frame(
                    result_position,
                    rgb_message.header.frame_id,
                    rgb_message.header.stamp,
                )
            except (tf2_ros.TransformException, ValueError) as error:
                rospy.logwarn_throttle(2.0, "Cannot transform hazard to %s: %s", self.start_frame, error)
                candidate_diagnostics.update({
                    "source": result_source,
                    "candidate_state": "transform_failed",
                    "three_d_available": True,
                    "track_id": None,
                    "confirmed": False,
                })
                frame_diagnostics.append(candidate_diagnostics)
                annotations.append("3D transform failed")
                continue

            geometry_validated = bool(
                (geometry_validation is not None and geometry_validation.accepted)
                or (
                    livox_geometry_validation is not None
                    and livox_geometry_validation.accepted
                )
            )
            localization_confidence = (
                geometry_validation.confidence
                if geometry_validation is not None and geometry_validation.accepted
                else (
                    livox_geometry_validation.confidence
                    if livox_geometry_validation is not None
                    and livox_geometry_validation.accepted
                    else self.geometry_fallback_confidence_scale
                )
            )
            if result_source == "monocular":
                localization_confidence = self.monocular_confidence_scale
            confidence = self._detection_confidence(detection) * localization_confidence
            required_confirmation = (
                self.edge_confirmation_count
                if detection.touches_border else self.confirmation_count
            )
            track = self.tracker.update(
                HazardObservation(
                    position=position_start,
                    confidence=confidence,
                    timestamp=rgb_message.header.stamp.to_sec(),
                    localization_source=result_source,
                    geometry_validated=geometry_validated,
                    required_confirmation_count=required_confirmation,
                )
            )
            if track is None:
                annotations.append("rejected jump/finalized")
                candidate_state = "tracker_rejected"
            else:
                residual = geometry_diagnostics.get("sphere_fit_residual_m")
                annotations.append(
                    "id={} {} fit={} c={:.2f} d={} l={}".format(
                        track.id,
                        result_source,
                        "{:.3f}".format(residual) if residual is not None else "fallback",
                        confidence,
                        depth_estimate.valid_count if depth_estimate else 0,
                        livox_count,
                    )
                )
                candidate_state = "confirmed" if track.confirmed else "candidate"
            tracker_event = dict(getattr(self.tracker, "last_event", {}) or {})
            candidate_diagnostics.update({
                "source": result_source,
                "candidate_state": candidate_state,
                "three_d_available": True,
                "raw_camera_xyz": [float(value) for value in result_position],
                "start_frame_xyz": [float(value) for value in position_start],
                "geometry_validated": geometry_validated,
                "localization_confidence": localization_confidence,
                "combined_confidence": confidence,
                "required_confirmation_count": required_confirmation,
                "track_id": int(track.id) if track is not None else None,
                "confirmed": bool(track.confirmed) if track is not None else False,
                "tracker_event": tracker_event,
                "track_observation_count": (
                    int(track.observation_count) if track is not None else 0
                ),
                "track_validated_observation_count": (
                    int(track.validated_observation_count) if track is not None else 0
                ),
                "track_required_confirmation_count": (
                    int(track.required_confirmation_count)
                    if track is not None else int(required_confirmation)
                ),
                "track_confirmed": bool(track.confirmed) if track is not None else False,
                "track_position_xyz": (
                    [float(value) for value in track.position]
                    if track is not None else None
                ),
            })
            if track is not None:
                with self._diagnostics_lock:
                    self._track_diagnostics[int(track.id)] = dict(candidate_diagnostics)
            frame_diagnostics.append(candidate_diagnostics)

        self._publish_tracks(rgb_message.header.stamp)
        diagnostic_payload = {
            "stamp": rgb_message.header.stamp.to_sec(),
            "frame_id": rgb_message.header.frame_id,
            "image_width": int(rgb_info.width),
            "image_height": int(rgb_info.height),
            "camera_info": {
                "fx": float(rgb_info.K[0]), "fy": float(rgb_info.K[4]),
                "cx": float(rgb_info.K[2]), "cy": float(rgb_info.K[5]),
                "width": int(rgb_info.width), "height": int(rgb_info.height),
            },
            "candidate_count": len(frame_diagnostics),
            "candidates": frame_diagnostics,
        }
        diagnostic_payload.update(self._frame_audit_record(
            rgb_message.header.stamp, rgb_message.header.frame_id
        ))
        self.geometry_diagnostics_publisher.publish(
            String(data=json.dumps(diagnostic_payload, sort_keys=True, separators=(",", ":")))
        )
        if self.diagnostics_log_path:
            try:
                directory = os.path.dirname(self.diagnostics_log_path)
                if directory:
                    os.makedirs(directory, exist_ok=True)
                with open(self.diagnostics_log_path, "a", encoding="utf-8") as stream:
                    stream.write(json.dumps(diagnostic_payload, sort_keys=True) + "\n")
            except (OSError, TypeError, ValueError):
                rospy.logwarn_throttle(10.0, "Cannot append hazard diagnostics")
        if self.publish_debug_images:
            self._publish_debug(rgb_message, bgr, mask, detections, annotations)

    def _publish_debug(self, rgb_message, bgr, mask, detections, annotations):
        debug = self.detector.annotate(bgr, detections)
        for detection, annotation in zip(detections, annotations):
            cv2.putText(
                debug,
                annotation,
                (detection.x, min(debug.shape[0] - 5, detection.y + detection.height + 15)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.43,
                (255, 255, 0),
                1,
                cv2.LINE_AA,
            )
        mask_message = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
        mask_message.header = rgb_message.header
        debug_message = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_message.header = rgb_message.header
        self.mask_publisher.publish(mask_message)
        self.debug_publisher.publish(debug_message)

    def _publish_tracks(self, stamp=None):
        stamp = stamp or rospy.Time.now()
        tracks = list(self.tracker.visible_tracks(self.publish_candidates))
        array = Hazard3DArray()
        array.header.stamp = stamp
        array.header.frame_id = self.start_frame
        for track in tracks:
            hazard = Hazard3D()
            hazard.id = track.id
            hazard.position.x, hazard.position.y, hazard.position.z = track.position.tolist()
            hazard.confidence = track.average_confidence
            hazard.observation_count = track.observation_count
            hazard.confirmed = track.confirmed
            hazard.localization_source = track.localization_source
            array.hazards.append(hazard)
        self.hazard_publisher.publish(array)
        self.marker_publisher.publish(self._make_markers(tracks, stamp))

    def _make_markers(self, tracks, stamp):
        array = MarkerArray()
        clear = Marker()
        clear.header.stamp = stamp
        clear.header.frame_id = self.start_frame
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        for track in tracks:
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.start_frame
            marker.ns = "confirmed" if track.confirmed else "candidate"
            marker.id = track.id * 2
            marker.type = Marker.SPHERE if track.confirmed else Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = track.position.tolist()
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.32 if track.confirmed else 0.22
            if track.confirmed:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.1, 0.1
            else:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.7, 0.0
            marker.color.a = 0.9
            array.markers.append(marker)

            label = Marker()
            label.header = marker.header
            label.ns = marker.ns + "_text"
            label.id = marker.id + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = track.position[0]
            label.pose.position.y = track.position[1]
            label.pose.position.z = track.position[2] + 0.35
            label.pose.orientation.w = 1.0
            label.scale.z = 0.18
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = "#{} ({:.2f},{:.2f},{:.2f}) c={:.2f} n={} {}".format(
                track.id,
                track.position[0], track.position[1], track.position[2],
                track.average_confidence, track.observation_count,
                track.localization_source,
            )
            array.markers.append(label)
        return array

    def _timer_callback(self, _event):
        now = rospy.Time.now()
        self.tracker.expire_candidates(now.to_sec())
        if self.start_frame_ready:
            self._publish_tracks(now)

    def _finalize_callback(self, _request):
        if self._finalize_paths is not None:
            json_path, csv_path = self._finalize_paths
            return TriggerResponse(
                success=True,
                message="Already finalized: {}{}".format(
                    json_path, " CSV=" + csv_path if csv_path else ""
                ),
            )
        tracks = self.tracker.finalize()
        try:
            with self._diagnostics_lock:
                diagnostics = dict(self._track_diagnostics)
            self._finalize_paths = self.result_manager.save(tracks, diagnostics)
        except (OSError, ValueError) as error:
            rospy.logerr("Cannot save finalized hazards: %s", error)
            return TriggerResponse(success=False, message=str(error))
        self._publish_tracks(rospy.Time.now())
        json_path, csv_path = self._finalize_paths
        rospy.loginfo("Finalized %d hazards to %s", len(tracks), json_path)
        return TriggerResponse(
            success=True,
            message="Saved {} confirmed hazards to {}{}".format(
                len(tracks), json_path, " and " + csv_path if csv_path else ""
            ),
        )


def main():
    rospy.init_node("hazard_pipeline")
    try:
        HazardPipelineNode()
    except (KeyError, TypeError, ValueError) as error:
        rospy.logfatal("Invalid hazard pipeline configuration: %s", error)
        raise SystemExit(2)
    rospy.spin()


if __name__ == "__main__":
    main()
