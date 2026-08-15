#!/usr/bin/env python3
"""
PointCloud to LaserScan converter for tilted Livox Mid-360.

Transforms every point from tilted laser_livox frame to horizontal base frame
first, then computes polar coordinates in the base frame's XY plane.

Publishes /scan_laser (LaserScan, frame=base) with horizontally-correct ranges.

DIRECTIONAL BLIND CONE: When the robot tilts beyond max_tilt (stairs, rough
terrain), forward-facing rays are invalidated because the tilted sensor sees
ground as false obstacles. Side and rear rays remain valid for AMCL.
Scans are NEVER fully dropped — AMCL needs continuous /scan_laser.
"""

import math
import rospy
import tf
from sensor_msgs.msg import PointCloud, LaserScan


class PointCloudToLaserScan:
    def __init__(self):
        rospy.init_node("pointcloud_to_laserscan", anonymous=False)

        # ---- Sensor geometry: laser_livox -> base ----
        self.laser_x = rospy.get_param("~laser_x", 0.2)
        self.laser_z = rospy.get_param("~laser_z", 0.08)
        self.pitch = rospy.get_param("~laser_pitch", 0.785)
        self.cos_p = math.cos(self.pitch)
        self.sin_p = math.sin(self.pitch)

        # ---- Ground filter (two modes) ----
        # Flat (tilt < max_tilt): low margin captures 0.15m-radius obstacles
        #   threshold = -0.31 + 0.12 = -0.19
        #   0.30m obstacle top bz≈-0.01, center bz≈-0.16 → both > -0.19 → DETECTED
        # Tilted (tilt > max_tilt): conservative margin prevents false ground
        #   threshold = -0.31 + 0.40 = +0.09
        #   Only 0.40m+ obstacles detected; ground/stair surfaces safely ignored
        self.ground_z = rospy.get_param("~ground_z", -0.31)
        self.ground_margin = rospy.get_param("~ground_margin", 0.12)
        self.ground_margin_tilted = rospy.get_param("~ground_margin_tilted", 0.40)
        self.ceiling_z = rospy.get_param("~ceiling_z", 2.0)
        self.max_v_angle = rospy.get_param("~max_v_angle", 0.70)  # ~40 deg

        # ---- Tilt + blind cone ----
        # When base tilt > max_tilt, two protections activate:
        #   1. Forward rays blinded (proportional cone: 0°→60° half-angle)
        #   2. ALL remaining rays use ground_margin_tilted (conservative)
        # Together: forward false ground removed + side rays tolerant of stairs
        self.max_tilt = rospy.get_param("~max_tilt", 0.10)             # rad, ~5.7°
        self.blind_cone_half = rospy.get_param("~blind_cone_half", 1.047)  # rad, 60° max
        self.blind_cone_ramp = rospy.get_param("~blind_cone_ramp", 0.175)  # rad, 10° ramp
        self.tf_listener = tf.TransformListener()
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base")

        # ---- Scan parameters ----
        self.angle_min = rospy.get_param("~angle_min", -math.pi)
        self.angle_max = rospy.get_param("~angle_max", math.pi)
        self.angle_increment = rospy.get_param("~angle_increment", math.pi / 180.0)
        self.range_min = rospy.get_param("~range_min", 0.15)
        self.range_max = rospy.get_param("~range_max", 30.0)
        self.scan_time = rospy.get_param("~scan_time", 0.1)
        self.output_frame = "base"
        self.input_topic = rospy.get_param("~input_topic", "/scan")
        self.output_topic = rospy.get_param("~output_topic", "/scan_laser")

        self.num_bins = int(round((self.angle_max - self.angle_min) / self.angle_increment))
        if self.num_bins <= 0:
            self.num_bins = 1

        self.sub = rospy.Subscriber(self.input_topic, PointCloud, self.cloud_callback, queue_size=10)
        self.pub = rospy.Publisher(self.output_topic, LaserScan, queue_size=10)
        self.pub_raw = rospy.Publisher("/scan_laser_raw", LaserScan, queue_size=10)

        rospy.loginfo(
            "pointcloud_to_laserscan: %s -> %s, output_frame=%s, "
            "ground_z=%.2f margin=%.2f ceiling=%.2f v_angle=%.0fdeg "
            "max_tilt=%.1fdeg blind_cone=%.0fdeg ramp=%.0fdeg, %d bins",
            self.input_topic, self.output_topic, self.output_frame,
            self.ground_z, self.ground_margin, self.ceiling_z,
            math.degrees(self.max_v_angle),
            math.degrees(self.max_tilt), math.degrees(self.blind_cone_half),
            math.degrees(self.blind_cone_ramp), self.num_bins
        )

    # ------------------------------------------------------------------
    # Coordinate transform
    # ------------------------------------------------------------------

    def to_base(self, x, y, z):
        """
        Transform point from laser_livox frame to base frame.

        laser_livox -> base:
          1. rotate by +pitch around Y axis
          2. translate by [laser_x, 0, laser_z]
        """
        rx = self.cos_p * x + self.sin_p * z
        ry = y
        rz = -self.sin_p * x + self.cos_p * z
        bx = rx + self.laser_x
        by = ry
        bz = rz + self.laser_z
        return bx, by, bz

    # ------------------------------------------------------------------
    # Tilt detection
    # ------------------------------------------------------------------

    def _get_tilt(self, stamp):
        """
        Compute base frame tilt from odom→base TF.

        Returns (tilt_rad, pitch_rad) or (None, None) on TF failure.
        tilt: combined angle from vertical (always >= 0).
        pitch: signed pitch (positive = nose-up, negative = nose-down).
        """
        try:
            self.tf_listener.waitForTransform(
                self.odom_frame, self.base_frame, stamp,
                rospy.Duration(0.05))
            (trans, rot) = self.tf_listener.lookupTransform(
                self.odom_frame, self.base_frame, stamp)
            x, y, z, w = rot
            sinr = 2.0 * (w * x + y * z)
            cosr = 1.0 - 2.0 * (x * x + y * y)
            roll = math.atan2(sinr, cosr)
            sinp = 2.0 * (w * y - z * x)
            pitch = math.asin(max(-1.0, min(1.0, sinp)))
            tilt = math.acos(math.cos(roll) * math.cos(pitch))
            return tilt, pitch
        except (tf.Exception, tf.LookupException,
                tf.ConnectivityException, tf.ExtrapolationException):
            return None, None

    # ------------------------------------------------------------------
    # Point cloud processing
    # ------------------------------------------------------------------

    def cloud_callback(self, cloud_msg):
        points = cloud_msg.points
        if not points:
            return

        # ---- Detect tilt ----
        stamp = (cloud_msg.header.stamp
                 if cloud_msg.header.stamp != rospy.Time(0)
                 else rospy.Time(0))
        tilt, pitch = self._get_tilt(stamp)

        blind_active = tilt is not None and tilt > self.max_tilt

        # Two-mode ground filter:
        #   Flat: aggressive margin (0.25) → detect 0.30m obstacles
        #   Tilted: conservative margin (0.40) → ignore stair surfaces
        if blind_active:
            effective_margin = self.ground_margin_tilted
        else:
            effective_margin = self.ground_margin

        ranges_obs = [float('inf')] * self.num_bins
        ranges_raw = [float('inf')] * self.num_bins

        obs_count = 0
        total_kept = 0
        blind_dropped = 0
        threshold = self.ground_z + effective_margin

        for pt in points:
            # 1. Transform to base frame
            bx, by, bz = self.to_base(pt.x, pt.y, pt.z)

            # 2. Filter by vertical angle (drop overhead rays)
            dx, dy, dz = bx - self.laser_x, by, bz - self.laser_z
            horiz = math.sqrt(dx * dx + dy * dy)
            if horiz < 0.001:
                continue
            v_angle = abs(math.atan2(dz, horiz))
            if v_angle > self.max_v_angle:
                continue

            # 3. Polar coordinates from base origin
            h_range = math.sqrt(bx * bx + by * by)
            if h_range < self.range_min or h_range > self.range_max:
                continue
            h_angle = math.atan2(by, bx)

            # 4. Bin index
            bin_idx = int(round((h_angle - self.angle_min) / self.angle_increment))
            if not (0 <= bin_idx < self.num_bins):
                continue

            total_kept += 1

            # Raw: all points (debug)
            if h_range < ranges_raw[bin_idx]:
                ranges_raw[bin_idx] = h_range

            # ---- Directional blind cone (proportional to tilt) ----
            # When tilted, forward-facing rays hit ground → false obstacles.
            # Blind cone grows linearly from 0° at max_tilt to full
            # blind_cone_half at max_tilt + blind_cone_ramp.
            if blind_active:
                excess = tilt - self.max_tilt
                ratio = min(1.0, excess / self.blind_cone_ramp)
                effective_half = self.blind_cone_half * ratio
                if abs(h_angle) < effective_half:
                    blind_dropped += 1
                    continue  # skip obstacle check for this ray

            # 5. Obstacle: above ground threshold AND below ceiling
            if bz > threshold and bz < self.ceiling_z:
                if h_range < ranges_obs[bin_idx]:
                    ranges_obs[bin_idx] = h_range
                    obs_count += 1

        # Heartbeat
        blind_str = ""
        if blind_active:
            excess = tilt - self.max_tilt
            ratio = min(1.0, excess / self.blind_cone_ramp)
            eff = math.degrees(self.blind_cone_half * ratio)
            blind_str = " BLIND(±%.0f° marg=%.2f, %d dropped)" % (
                eff, effective_margin, blind_dropped)
        tilt_str = ""
        if tilt is not None:
            tilt_str = " tilt=%.1f° pitch=%.1f°" % (
                math.degrees(tilt), math.degrees(pitch))
        rospy.loginfo_throttle(
            5.0,
            "pc2laser: %d pts -> %d kept, %d obstacle bins%s%s",
            len(points), total_kept, obs_count, tilt_str, blind_str
        )

        self._publish(cloud_msg, ranges_obs, self.pub)
        self._publish(cloud_msg, ranges_raw, self.pub_raw)

    def _publish(self, cloud_msg, ranges, publisher):
        ranges = [r if r != float('inf') else self.range_max for r in ranges]
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = self.output_frame
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = self.scan_time
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        scan.intensities = []
        publisher.publish(scan)


if __name__ == "__main__":
    try:
        PointCloudToLaserScan()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
