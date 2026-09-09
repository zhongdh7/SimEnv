#!/usr/bin/env python3
"""Low small floor-obstacle RGB-D detector for the avoidance layer.

SimEnv plants small low obstacles -- danger red spheres (r=0.15), red distractor
boxes (0.30x0.30x0.30) and green distractor spheres (r=0.15) -- whose centres
sit at floor+0.15.  They are too low/small for the Livox-only occupancy map
(scan_min_range=2.0 and the lidar sits above them), so neither the DFS planner
nor collision_safety ever sees them and the robot drives over / trips on them.

This node is deliberately independent of the red-sphere *hazard reporting*
pipeline (hazard_perception detector/tracker/finalize stay untouched) and of
the navigation / map-building code.  It detects saturated RED or GREEN floor
blobs (verified to be unique to those three obstacle kinds in this building),
localises them in the odom frame and publishes them as a PointCloud2 that
collision_safety consumes as a blocking obstacle.

RGB/depth are aligned 640x480 @10 Hz; image topic headers are labelled with the
sensor BODY frame ``real_sense`` while pixels use ROS optical axes, so detected
pixels are converted with optical_to_sensor_body before the odom TF (same
convention as hazard_pipeline_node).
"""

import math

import cv2
import message_filters
import numpy as np
import rospy
import tf2_geometry_msgs  # noqa: F401 - registers PointStamped/pose converters with tf2
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from hazard_perception.localization import (
    CameraIntrinsics,
    backproject_pixel,
    depth_to_metres,
    optical_to_sensor_body,
)


def _hsv_in_range(hsv, hue_lo, hue_hi, sat_lo, sat_hi, val_lo, val_hi):
    lower = np.array([hue_lo, sat_lo, val_lo], dtype=np.uint8)
    upper = np.array([hue_hi, sat_hi, val_hi], dtype=np.uint8)
    return cv2.inRange(hsv, lower, upper)


class LowObstacleNode:
    def __init__(self):
        self.bridge = CvBridge()
        get = rospy.get_param

        self.rgb_topic = get("~rgb_topic", "/real_sense/rgb/image_raw")
        self.depth_topic = get("~depth_topic", "/real_sense/depth/image_raw")
        self.rgb_info_topic = get("~rgb_info_topic", "/real_sense/rgb/camera_info")
        self.depth_info_topic = get("~depth_info_topic", "/real_sense/depth/camera_info")
        self.camera_frame = get("~camera_frame", "real_sense")
        self.odom_frame = get("~odom_frame", "odom")
        self.base_frame = get("~base_frame", "base")

        # Colour thresholds (OpenCV hue is 0..179).  Pure red and pure green
        # appear only on the three obstacle kinds; furniture planters are dark
        # green with low saturation and are excluded by the sat/value gates.
        self.red_lo_1 = int(get("~red_hue_lo_1", 0))
        self.red_hi_1 = int(get("~red_hue_hi_1", 10))
        self.red_lo_2 = int(get("~red_hue_lo_2", 170))
        self.red_hi_2 = int(get("~red_hue_hi_2", 179))
        self.green_lo = int(get("~green_hue_lo", 45))
        self.green_hi = int(get("~green_hue_hi", 85))
        # Red saturation gate matches hazard_perception's proven red-sphere
        # detector (sat>=100); pure red appears only on danger spheres/boxes.
        # Green uses a higher gate so dark-green planters (sat~100) never become
        # phantom obstacles while the pure-green distractor sphere (sat 255)
        # still passes comfortably.
        self.red_sat_min = int(get("~red_sat_min", 100))
        self.green_sat_min = int(get("~green_sat_min", 120))
        self.sat_max = int(get("~sat_max", 255))
        self.val_min = int(get("~val_min", 70))
        self.val_max = int(get("~val_max", 255))

        self.min_area = float(get("~min_area", 120.0))
        self.max_area = float(get("~max_area", 150000.0))
        self.border_margin = int(get("~border_margin", 3))

        # Depth and world-size gates.  Obstacles span roughly 0.12-0.5 m in the
        # world; larger/distant red-green structure is ignored.
        self.min_depth = float(get("~min_depth", 0.4))
        self.max_depth = float(get("~max_depth", 4.5))
        self.min_valid_pixels = int(get("~min_valid_pixels", 25))
        self.obj_min_m = float(get("~obj_min_m", 0.08))
        self.obj_max_m = float(get("~obj_max_m", 0.60))

        self.confirmation_count = int(get("~confirmation_count", 2))
        self.merge_distance = float(get("~merge_distance", 0.5))
        self.track_stale_s = float(get("~track_stale_s", 0.6))

        self.out_topic = get("~out_topic", "/hazard_perception/low_obstacles")
        self.marker_topic = get("~marker_topic", "/hazard_perception/low_obstacles_markers")
        self.debug_topic = get("~debug_topic", "/hazard_perception/low_obstacles_debug")
        self.publish_debug = bool(get("~publish_debug", False))

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.out_pub = rospy.Publisher(self.out_topic, PointCloud2, queue_size=1)
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)
        self.debug_pub = rospy.Publisher(self.debug_topic, Image, queue_size=1)

        rgb = message_filters.Subscriber(self.rgb_topic, Image, queue_size=1)
        depth = message_filters.Subscriber(self.depth_topic, Image, queue_size=1)
        rgb_info = message_filters.Subscriber(self.rgb_info_topic, CameraInfo, queue_size=1)
        depth_info = message_filters.Subscriber(self.depth_info_topic, CameraInfo, queue_size=1)
        sync = message_filters.ApproximateTimeSynchronizer(
            [rgb, depth, rgb_info, depth_info],
            queue_size=8,
            slop=0.08,
            allow_headerless=False,
        )
        sync.registerCallback(self._on_sync)

        self._tracks = []  # list of dicts: pos(odom xy), last_seen, count
        rospy.loginfo(
            "low_obstacle_node: rgb=%s depth=%s red_sat_min=%d "
            "green_sat_min=%d val_min=%d out=%s", self.rgb_topic,
            self.depth_topic, self.red_sat_min, self.green_sat_min,
            self.val_min, self.out_topic)

    # ------------------------------------------------------------------ utils
    def _to_odom(self, optical_xyz, stamp):
        """Convert an optical-axis point to the odom frame.

        The pixel stream is optical but its header frame is the sensor body
        frame ``real_sense``; mirror hazard_pipeline's optical_to_sensor_body
        conversion before the ordinary odom transform.
        """
        body = optical_to_sensor_body(optical_xyz)
        point = PointStamped()
        point.header.stamp = stamp
        point.header.frame_id = self.camera_frame
        point.point = Point(x=float(body[0]), y=float(body[1]), z=float(body[2]))
        transformed = self.tf_buffer.transform(point, self.odom_frame,
                                               rospy.Duration(0.15))
        return (float(transformed.point.x), float(transformed.point.y),
                float(transformed.point.z))

    def _color_mask(self, hsv):
        red = cv2.bitwise_or(
            _hsv_in_range(hsv, self.red_lo_1, self.red_hi_1,
                          self.red_sat_min, self.sat_max, self.val_min, self.val_max),
            _hsv_in_range(hsv, self.red_lo_2, self.red_hi_2,
                          self.red_sat_min, self.sat_max, self.val_min, self.val_max))
        green = _hsv_in_range(hsv, self.green_lo, self.green_hi,
                              self.green_sat_min, self.sat_max,
                              self.val_min, self.val_max)
        return cv2.bitwise_or(red, green)

    # ------------------------------------------------------------------ detect
    def _detect(self, bgr, depth, info, stamp):
        intrinsics = CameraIntrinsics.from_camera_info(info)
        intrinsics.validate()
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = self._color_mask(hsv)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours_result = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
        contours = contours_result[-2]  # OpenCV 3/4 return-value compatibility
        height, width = depth.shape[:2]
        results = []  # odom (x, y, z)
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.min_area or area > self.max_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            # Reject blobs clipped at the left/right/top edges (partial, centroid
            # unreliable).  The bottom edge is deliberately allowed to clip: a
            # low floor obstacle seen from the ~0.4 m level camera leaves the
            # frame at the bottom exactly when the robot is closest -- the case
            # collision_safety must act on.
            if (x <= self.border_margin or y <= self.border_margin or
                    x + w >= width - self.border_margin):
                continue
            # A floor object below the ~0.4 m level camera lives entirely in the
            # lower half of the image; anything crossing the optical axis is not
            # one of these low obstacles.
            cy = float(intrinsics.cy)
            if y + h / 2.0 <= cy:
                continue

            roi = mask[y:y + h, x:x + w]
            droi = depth[y:y + h, x:x + w]
            valid = (roi > 0) & np.isfinite(droi) & (droi >= self.min_depth) \
                & (droi <= self.max_depth) & (droi > 0.0)
            count = int(np.count_nonzero(valid))
            if count < self.min_valid_pixels:
                continue
            rows, cols = np.nonzero(valid)
            vals = droi[valid].astype(np.float64)
            depth_median = float(np.median(vals))
            # vertical / horizontal world span from the median depth.
            span_v = float(np.max(rows) - np.min(rows) + 1)
            span_u = float(np.max(cols) - np.min(cols) + 1)
            world_v = span_v * depth_median / intrinsics.fy
            world_u = span_u * depth_median / intrinsics.fx
            if not (self.obj_min_m <= world_v <= self.obj_max_m and
                    self.obj_min_m <= world_u <= self.obj_max_m):
                continue

            u = float(np.median(cols)) + x
            v = float(np.median(rows)) + y
            optical = backproject_pixel(u, v, depth_median, intrinsics)
            try:
                results.append(self._to_odom(optical, stamp))
            except (tf2_ros.TransformException, ValueError) as error:
                rospy.logwarn_throttle(5.0, "low_obstacle transform: %s", error)
        return mask, results

    # ------------------------------------------------------------------ track
    def _update_tracks(self, detections, now):
        # Merge detections into tracks in the odom plane (z kept as the object's
        # true odom height so the safety layer can floor-gate it).
        for x, y, z in detections:
            best = None
            best_d = None
            for track in self._tracks:
                d = math.hypot(track["x"] - x, track["y"] - y)
                if d <= self.merge_distance and (best_d is None or d < best_d):
                    best = track
                    best_d = d
            if best is not None:
                best["x"] = x
                best["y"] = y
                best["z"] = z
                best["last"] = now
                best["count"] += 1
            else:
                self._tracks.append({"x": x, "y": y, "z": z, "last": now,
                                     "count": 1})
        # Remove stale tracks.
        alive = []
        for track in self._tracks:
            if now - track["last"] <= self.track_stale_s:
                alive.append(track)
        self._tracks = alive
        confirmed = [t for t in self._tracks if t["count"] >= self.confirmation_count]
        return confirmed

    # ------------------------------------------------------------------ pubs
    def _publish_cloud(self, tracks, stamp):
        header = Header()
        header.stamp = stamp
        header.frame_id = self.odom_frame
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        points = [(t["x"], t["y"], t["z"]) for t in tracks]
        cloud = point_cloud2.create_cloud(header, fields, points)
        self.out_pub.publish(cloud)
        self._publish_markers(tracks, stamp)

    def _publish_markers(self, tracks, stamp):
        array = MarkerArray()
        clear = Marker()
        clear.header.stamp = stamp
        clear.header.frame_id = self.odom_frame
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        for idx, t in enumerate(tracks):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.odom_frame
            marker.ns = "low_obstacle"
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = t["x"]
            marker.pose.position.y = t["y"]
            marker.pose.position.z = 0.15
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.30
            marker.scale.y = 0.30
            marker.scale.z = 0.30
            marker.color.r = 1.0
            marker.color.g = 0.6
            marker.color.b = 0.0
            marker.color.a = 0.8
            array.markers.append(marker)
        self.marker_pub.publish(array)

    def _publish_debug(self, bgr, mask, stamp):
        debug = bgr.copy()
        debug[mask > 0] = (0, 255, 0)
        try:
            message = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            message.header.stamp = stamp
            self.debug_pub.publish(message)
        except CvBridgeError:
            pass

    # ------------------------------------------------------------------ cb
    def _on_sync(self, rgb_msg, depth_msg, rgb_info, _depth_info):
        try:
            bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            raw_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            depth = depth_to_metres(np.asarray(raw_depth), depth_msg.encoding)
        except (CvBridgeError, ValueError) as error:
            rospy.logwarn_throttle(2.0, "low_obstacle decode: %s", error)
            return
        stamp = rgb_msg.header.stamp
        mask, detections = self._detect(bgr, depth, rgb_info, stamp)
        now = stamp.to_sec()
        tracks = self._update_tracks(detections, now)
        self._publish_cloud(tracks, stamp)
        if self.publish_debug:
            self._publish_debug(bgr, mask, stamp)


def main():
    rospy.init_node("low_obstacle_node")
    LowObstacleNode()
    rospy.spin()


if __name__ == "__main__":
    main()
