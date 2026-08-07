#!/usr/bin/env python3
"""
PointCloud to LaserScan converter for tilted Livox Mid-360.

THE KEY FIX: transforms every point from tilted laser_livox frame to
horizontal base frame FIRST, then computes polar coordinates in the
base frame's horizontal XY plane. The raw sqrt(x^2+y^2) in the tilted
laser frame is geometrically meaningless for a 2D LaserScan.

Publishes /scan_laser (LaserScan, frame=base) with horizontally-correct ranges.
"""

import math
import rospy
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

        # ---- Vertical filter (ground + ceiling + angle) ----
        # ground_z: nominal ground Z in base frame (TF: 0.313m above floor)
        #   Obstacle threshold = -0.31 + 0.35 = 0.04m (base frame)
        #   Stair-safe: ~12° tilt @2m before ground triggers as obstacle.
        #   NOTE: red sphere/box (0.30m tall, top base_z=-0.01m) is BELOW
        #   this threshold and will be filtered. That is a hardware limit.
        self.ground_z = rospy.get_param("~ground_z", -0.35)
        self.ground_margin = rospy.get_param("~ground_margin", 0.35)
        self.ceiling_z = rospy.get_param("~ceiling_z", 2.0)
        # Max vertical angle from laser (rad). Rays pointing too high
        # (ceiling, overhead structures, hollow stair gaps) are dropped.
        self.max_v_angle = rospy.get_param("~max_v_angle", 0.44)  # ~25 deg

        # ---- Scan parameters ----
        self.angle_min = rospy.get_param("~angle_min", -math.pi)
        self.angle_max = rospy.get_param("~angle_max", math.pi)
        self.angle_increment = rospy.get_param("~angle_increment", math.pi / 180.0)
        self.range_min = rospy.get_param("~range_min", 0.15)
        self.range_max = rospy.get_param("~range_max", 30.0)
        self.scan_time = rospy.get_param("~scan_time", 0.1)
        self.output_frame = "base"  # fixed: scan in horizontal base frame
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
            "ground_z=%.2f margin=%.2f ceiling=%.2f v_angle=%.0fdeg, %d bins",
            self.input_topic, self.output_topic, self.output_frame,
            self.ground_z, self.ground_margin, self.ceiling_z,
            math.degrees(self.max_v_angle), self.num_bins
        )

    def to_base(self, x, y, z):
        """
        Transform point from laser_livox frame to base frame.

        laser_livox -> base:
          1. rotate by +pitch around Y axis
          2. translate by [laser_x, 0, laser_z]
        """
        # Rotate by +pitch around Y
        rx = self.cos_p * x + self.sin_p * z
        ry = y
        rz = -self.sin_p * x + self.cos_p * z
        # Translate
        bx = rx + self.laser_x
        by = ry
        bz = rz + self.laser_z
        return bx, by, bz

    def cloud_callback(self, cloud_msg):
        points = cloud_msg.points
        if not points:
            return

        ranges_obs = [float('inf')] * self.num_bins  # filtered (obstacles only)
        ranges_raw = [float('inf')] * self.num_bins  # unfiltered (all points)

        obs_count = 0
        total_kept = 0
        for pt in points:
            # 1. Transform to horizontal base frame
            bx, by, bz = self.to_base(pt.x, pt.y, pt.z)

            # 2. Compute vertical angle from laser (filter overhead rays)
            dx, dy, dz = bx - self.laser_x, by, bz - self.laser_z
            horiz = math.sqrt(dx * dx + dy * dy)
            if horiz < 0.001:
                continue
            v_angle = abs(math.atan2(dz, horiz))
            if v_angle > self.max_v_angle:
                continue  # skip rays pointing too high (ceiling, overhead)

            # 3. Compute horizontal polar from BASE ORIGIN
            h_range = math.sqrt(bx * bx + by * by)
            if h_range < self.range_min or h_range > self.range_max:
                continue
            h_angle = math.atan2(by, bx)

            # 4. Map to bin
            bin_idx = int(round((h_angle - self.angle_min) / self.angle_increment))
            if not (0 <= bin_idx < self.num_bins):
                continue

            total_kept += 1

            # Raw: all points (debug)
            if h_range < ranges_raw[bin_idx]:
                ranges_raw[bin_idx] = h_range

            # Obstacle: point above ground but below ceiling
            if (bz > self.ground_z + self.ground_margin and
                bz < self.ceiling_z):
                if h_range < ranges_obs[bin_idx]:
                    ranges_obs[bin_idx] = h_range
                    obs_count += 1

        # Rate-limited heartbeat: confirms data flow every 5 seconds
        rospy.loginfo_throttle(
            5.0,
            "pc2laser: %d pts -> %d kept, %d obstacle bins filled",
            len(points), total_kept, obs_count
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
