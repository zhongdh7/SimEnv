#!/usr/bin/env python3
"""
Stair Climber — direct cmd_vel control for stair ascent/descent.

Port of the stair-climbing logic from explore_full.  When triggered the node
takes over /cmd_vel, executes an S-shaped waypoint sequence through the stair
volume, detects the floor transition via z-height, and signals completion.

Geometry is loaded from layout_metadata.json (same file explore_full uses).

Trigger:  /stair_climb/goal  (std_msgs/String — "up" or "down")
Feedback: /stair_climb/status (std_msgs/String)
Result:   /stair_climb/result (std_msgs/String — "success" or "failure")
"""

import json
import math
import sys

import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


# ---------------------------------------------------------------------------
# Stair geometry — extracted from explore_full
# ---------------------------------------------------------------------------

class StairGeometry(object):
    """Compute stair waypoints from the bounds stored in layout_metadata."""

    def __init__(self, stair_bounds, floor_height=2.6, floor_count=3):
        self.stair_bounds = stair_bounds
        self.floor_height = float(floor_height)
        self.floor_count = int(floor_count)
        self._precompute()

    def _precompute(self):
        bounds = self.stair_bounds
        width = float(bounds["x_max"]) - float(bounds["x_min"])
        # These magic constants are from explore_full's _stair_geometry()
        self.x_left = float(bounds["x_min"]) + 0.26 * width
        self.x_right = float(bounds["x_max"]) - 0.26 * width
        self.x_exit = float(bounds["x_max"]) + 0.5 * width
        self.y_entry = float(bounds["y_min"]) + 0.65
        self.y_turn = float(bounds["y_max"]) - 1.65

    def route(self, direction):
        """Return the ordered waypoint list for one flight.

        direction: "up" or "down"
        """
        if direction == "up":
            return [
                (self.x_left,  self.y_entry),
                (self.x_left,  self.y_turn),
                (self.x_right, self.y_turn),
                (self.x_right, self.y_entry),
                (self.x_exit,  self.y_entry),
            ]
        # down — reverse the S-curve
        return [
            (self.x_right, self.y_entry),
            (self.x_right, self.y_turn),
            (self.x_left,  self.y_turn),
            (self.x_left,  self.y_entry),
            (self.x_exit,  self.y_entry),
        ]

    def floor_index(self, z, home_z):
        """Which floor the robot is currently on."""
        if self.floor_height <= 0.0:
            return 0
        relative = (float(z) - float(home_z)) / self.floor_height
        return clamp(int(round(relative)), 0, self.floor_count - 1)


# ---------------------------------------------------------------------------
# Waypoint-tracking controller
# ---------------------------------------------------------------------------

class StairController(object):
    """Follow a list of (x, y) waypoints with direct cmd_vel commands."""

    # ---- tunable ----
    WAYPOINT_TOL = 0.25       # metre — tight, from explore_full
    FINISH_TOL = 0.40         # metre — finish-gate tolerance
    STAIR_SPEED = 0.80        # m/s max linear
    MAX_LATERAL = 0.15        # m/s max lateral
    MAX_YAW_RATE = 1.0        # rad/s max rotation
    LINEAR_GAIN = 1.0
    CONTROL_RATE = 20.0       # Hz
    LOOKAHEAD = 1.20          # metre — limit on-stair advance
    STALE_TIMEOUT = 2.0       # seconds without odometry before abort
    COMPLETION_HOLD = 1.5     # seconds to hold zero velocity after finishing

    def __init__(self, geometry):
        self.geo = geometry
        self.pose = None          # (x, y, z, yaw)
        self.home_z = None
        self.active = False
        self._route = []
        self._index = 0
        self._target_floor = None
        self._finish_time = None

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.status_pub = rospy.Publisher(
            "/stair_climb/status", String, queue_size=5
        )
        self.result_pub = rospy.Publisher(
            "/stair_climb/result", String, queue_size=1, latch=True
        )

        rospy.Subscriber("/Odometry_gazebo", Odometry, self._odom_cb, queue_size=5)
        rospy.Subscriber("/stair_climb/goal", String, self._goal_cb, queue_size=2)

        self._last_odom = rospy.Time.now()
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / self.CONTROL_RATE), self._control_cb
        )

        rospy.loginfo("StairController ready: x=[%.2f,%.2f] y=[%.2f,%.2f]",
                      self.geo.stair_bounds["x_min"],
                      self.geo.stair_bounds["x_max"],
                      self.geo.stair_bounds["y_min"],
                      self.geo.stair_bounds["y_max"])

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _odom_cb(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (
                pos.x, pos.y, pos.z,
                ori.x, ori.y, ori.z, ori.w)):
            return
        yaw = tf.transformations.euler_from_quaternion(
            (ori.x, ori.y, ori.z, ori.w))[2]
        self.pose = (float(pos.x), float(pos.y), float(pos.z), float(yaw))
        if self.home_z is None:
            self.home_z = float(pos.z)
            rospy.loginfo("home z = %.3f", self.home_z)
        self._last_odom = rospy.Time.now()

    def _goal_cb(self, msg):
        if self.active:
            rospy.logwarn("stair climber already active, ignoring %s", msg.data)
            return
        direction = msg.data.strip().lower()
        if direction not in ("up", "down"):
            rospy.logerr("invalid stair goal '%s' — use 'up' or 'down'", msg.data)
            self.result_pub.publish("failure: invalid direction")
            return
        self._start(direction)

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------

    def _start(self, direction):
        if self.pose is None or self.home_z is None:
            rospy.logerr("cannot start stair climb — no odometry yet")
            self.result_pub.publish("failure: no odometry")
            return

        self._route = self.geo.route(direction)
        self._index = 0
        current_floor = self.geo.floor_index(self.pose[2], self.home_z)
        self._target_floor = (
            current_floor + 1 if direction == "up" else current_floor - 1
        )
        self._target_floor = max(
            0, min(self.geo.floor_count - 1, self._target_floor)
        )
        self.active = True
        self._finish_time = None

        rospy.loginfo("stair climb START: direction=%s current_floor=%d "
                      "target_floor=%d home_z=%.3f z=%.3f route=%d waypoints",
                      direction, current_floor, self._target_floor,
                      self.home_z, self.pose[2], len(self._route))
        self.status_pub.publish(
            "stairs_%s_started floor=%d->%d" % (
                direction, current_floor, self._target_floor
            ))

    def _finish(self, success, message):
        self.active = False
        self._route = []
        self._index = 0
        self._finish_time = None
        result = "success" if success else "failure"
        rospy.loginfo("stair climb %s: %s", result.upper(), message)
        self.result_pub.publish("%s: %s" % (result, message))
        self.status_pub.publish("stairs_%s" % result)

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_cb(self, _event):
        if not self.active:
            return

        # Stale check
        if rospy.Time.now() - self._last_odom > rospy.Duration(self.STALE_TIMEOUT):
            self._publish_zero()
            self._finish(False, "odometry timeout")
            return

        pose = self.pose
        route = self._route
        index = self._index

        # --- floor transition detection ---
        current_floor = self.geo.floor_index(pose[2], self.home_z)
        target = self._target_floor

        if current_floor == target:
            # We are on the target floor.  If we've also finished the
            # route (exited the stairs), declare success.
            if index >= len(route):
                if self._finish_time is None:
                    self._finish_time = rospy.Time.now()
                    self._publish_zero()
                    self.status_pub.publish("stairs_waiting_confirm floor=%d" % current_floor)
                elif (rospy.Time.now() - self._finish_time >
                      rospy.Duration(self.COMPLETION_HOLD)):
                    self._publish_zero()
                    self._finish(True, "reached floor %d" % target)
                else:
                    self._publish_zero()
                return
        elif target > current_floor and pose[2] < self.home_z + current_floor * self.geo.floor_height:
            # We're going up but z hasn't reached the next floor yet — normal
            pass
        elif target < current_floor and pose[2] > self.home_z + target * self.geo.floor_height + 1.3:
            # Going down but still too high — normal
            pass

        # --- waypoint advance ---
        while index < len(route):
            wp = route[index]
            dist = math.hypot(wp[0] - pose[0], wp[1] - pose[1])
            if dist < self.WAYPOINT_TOL:
                index += 1
            else:
                break
        self._index = index

        if index >= len(route):
            # Path complete — wait for floor confirmation
            if self._finish_time is None:
                self._finish_time = rospy.Time.now()
            self._publish_zero()
            self.status_pub.publish("stairs_route_complete waiting_floor=%d" % target)
            return

        # --- tracking waypoint ---
        waypoint = route[index]
        # Limit lookahead on stairs (from explore_full's _tracking_waypoint)
        if index in (1, 3):  # longitudinal segments on the stairs
            delta_y = waypoint[1] - pose[1]
            if abs(delta_y) > self.LOOKAHEAD:
                waypoint = (waypoint[0],
                            pose[1] + math.copysign(self.LOOKAHEAD, delta_y))

        dx = waypoint[0] - pose[0]
        dy = waypoint[1] - pose[1]
        body_x = math.cos(pose[3]) * dx + math.sin(pose[3]) * dy
        body_y = -math.sin(pose[3]) * dx + math.cos(pose[3]) * dy
        target_yaw = math.atan2(dy, dx)
        heading_error = target_yaw - pose[3]
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        cmd = Twist()
        cmd.linear.x = clamp(self.LINEAR_GAIN * body_x,
                             -self.STAIR_SPEED, self.STAIR_SPEED)
        cmd.linear.y = clamp(0.55 * body_y, -self.MAX_LATERAL, self.MAX_LATERAL)
        if abs(heading_error) > 1.0:
            cmd.linear.x *= 0.2
            cmd.linear.y *= 0.2
        cmd.angular.z = clamp(0.9 * heading_error,
                              -self.MAX_YAW_RATE, self.MAX_YAW_RATE)
        # Stair-mode speed caps (tight)
        cmd.linear.x = clamp(cmd.linear.x, -self.STAIR_SPEED, self.STAIR_SPEED)
        cmd.linear.y = clamp(cmd.linear.y, -self.MAX_LATERAL, self.MAX_LATERAL)
        cmd.angular.z = clamp(cmd.angular.z,
                              -self.MAX_YAW_RATE, self.MAX_YAW_RATE)

        if not all(math.isfinite(v) for v in (
                cmd.linear.x, cmd.linear.y, cmd.angular.z)):
            rospy.logerr_throttle(2.0, "stair climber computed NaN cmd_vel")
            cmd = Twist()

        self.cmd_pub.publish(cmd)
        self.status_pub.publish(
            "stairs_active wp=%d/%d floor=%d target=%d z=%.2f" % (
                index + 1, len(route), current_floor, target, pose[2]
            ))

    def _publish_zero(self):
        self.cmd_pub.publish(Twist())


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    rospy.init_node("stair_climber")

    # Load geometry — same source as explore_full
    layout_path = rospy.get_param(
        "~layout_metadata_path",
        "/home/loser/SimEnv/generated_building/layout_metadata.json",
    )
    floor_height = rospy.get_param("~floor_height", 2.6)
    floor_count = rospy.get_param("~floor_count", 3)

    try:
        with open(layout_path, "r") as fh:
            metadata = json.load(fh)
    except Exception as exc:
        rospy.logerr("cannot load layout metadata: %s", exc)
        sys.exit(1)

    floors = metadata.get("floors", [])
    if not floors:
        rospy.logerr("no floors in layout metadata")
        sys.exit(1)

    stair_bounds = dict(floors[0].get("stair_bounds", {}))
    if not stair_bounds:
        rospy.logerr("no stair_bounds in layout metadata")
        sys.exit(1)

    geometry = StairGeometry(stair_bounds, floor_height, floor_count)
    StairController(geometry)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
