#!/usr/bin/env python3
"""
Multi-Floor Explorer — DWA waypoint navigation + stair climbing for 3 floors.

Integrates the floor-0 waypoint sequence from auto_room_explorer with the
stair_climber node.  Each floor uses the same XY waypoints (identical layout).
Between floors the robot navigates to the stair entry, triggers stair climbing,
waits for completion, then resumes waypoint navigation on the new floor.

Flow:  floor-0 (22 wp) → stairs up → floor-1 (22 wp) → stairs up
    → floor-2 (22 wp) → stairs down × 2 → return home
"""

import json
import math

import rospy
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler


# ================================================================
# WAYPOINT SEQUENCE — same XY for every floor (layout is identical)
# ================================================================
FLOOR_WAYPOINTS = [
    # === Enter building → corridor midpoint ===
    ( 0.256, 14.641,  1.565),
    (-1.941, 14.722,  3.118),

    # === Left rooms (west wing) ===
    (-7.649, 20.250, -0.636),
    (-7.785,  9.343,  0.594),

    # === Back to corridor ===
    (-1.923, 14.477,  0.089),

    # === Right rooms (east wing) ===
    ( 2.387, 14.597,  0.000),
    ( 7.920, 19.999, -2.543),
    ( 8.142,  9.207,  2.550),

    # === Back to corridor ===
    ( 2.624, 14.554,  3.130),
    ( 0.290, 14.641, -3.121),

    # === Transit to far corridor ===
    ( 0.511, 28.162,  1.572),

    # === Far-left rooms ===
    (-2.365, 28.269,  3.120),
    (-7.255, 33.750, -0.652),
    (-7.291, 22.966,  0.623),

    # === Back to far corridor ===
    (-1.978, 28.193,  0.049),

    # === Far-right rooms ===
    ( 2.784, 28.069,  0.000),
    ( 8.394, 33.405, -2.587),
    ( 8.596, 22.847,  2.582),
    ( 8.549, 22.720,  2.323),

    # === Back to corridor & return ===
    ( 3.102, 27.996,  3.114),
    ( 0.440, 28.162, -3.131),
    ( 0.050,  1.997, -1.489),
    (-2.835,  1.801, -3.072),
]

# ---- stair approach positions ----
STAIR_ENTRY       = (-2.835,  1.801, -3.072)  # unified stair approach (up & down)
# After climbing, robot exits near:  (-0.05, 1.5)
STAIR_EXIT = (-0.05, 1.50)

# First waypoint after entering a new floor — corridor entrance
FLOOR_START_WP = (0.256, 14.641, 1.565)

# ---- home position ----
HOME = (0.0, -3.2, 1.5708)

# ================================================================
# TUNABLE
# ================================================================
GOAL_TIMEOUT = 120.0
GOAL_SEARCH_RADIUS = 2.0
STAIR_CLIMB_TIMEOUT = 300.0    # max seconds for one stair flight
POST_STAIR_SLEEP = 3.0         # settle time after stair climb


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class MultiFloorExplorer:
    """Orchestrate floor-by-floor exploration with stair transitions."""

    def __init__(self):
        rospy.init_node("multi_floor_explorer")

        # --- stair geometry from layout metadata ---
        layout_path = rospy.get_param(
            "~layout_metadata_path",
            "/home/loser/SimEnv/generated_building/layout_metadata.json",
        )
        self.floor_count = rospy.get_param("~floor_count", 3)
        self.floor_height = rospy.get_param("~floor_height", 2.6)

        self._load_stair_geometry(layout_path)

        # --- move_base action client ---
        rospy.loginfo("Waiting for move_base action server...")
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        if not self.ac.wait_for_server(rospy.Duration(30)):
            raise RuntimeError("move_base not available")
        rospy.loginfo("move_base connected")

        # --- costmap for goal validation ---
        self._costmap = None
        self._costmap_info = None
        self._costmap_sub = rospy.Subscriber(
            "/move_base/global_costmap/costmap",
            OccupancyGrid,
            self._costmap_cb,
        )
        deadline = rospy.Time.now() + rospy.Duration(15.0)
        while self._costmap is None and rospy.Time.now() < deadline:
            rospy.sleep(0.2)
        if self._costmap is None:
            rospy.logwarn("Costmap unavailable — goal adjust disabled")
        else:
            rospy.loginfo("Costmap ready (%dx%d %.3f m/pix)",
                          self._costmap_info.width,
                          self._costmap_info.height,
                          self._costmap_info.resolution)

        # --- odometry for floor detection ---
        self.pose = None
        self.home_z = None
        self.current_floor = 0
        rospy.Subscriber("/Odometry_gazebo", Odometry, self._odom_cb, queue_size=5)

        # --- stair climber communication ---
        self._stair_result = None
        self._stair_done = False
        rospy.Subscriber("/stair_climb/result", String, self._stair_result_cb)
        rospy.Subscriber("/stair_climb/status", String, self._stair_status_cb)
        self._stair_goal_pub = rospy.Publisher("/stair_climb/goal", String, queue_size=1)
        self._initpose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)

        # --- parameters ---
        self.goal_timeout = rospy.get_param("~goal_timeout", GOAL_TIMEOUT)
        self.search_radius = rospy.get_param("~goal_search_radius", GOAL_SEARCH_RADIUS)

        # --- state ---
        self._sent = 0
        self._ok = 0
        self._floors_completed = set()

    # ------------------------------------------------------------------
    # Initialization helpers
    # ------------------------------------------------------------------

    def _load_stair_geometry(self, layout_path):
        """Read stair bounds and pre-compute key positions."""
        try:
            with open(layout_path, "r") as fh:
                metadata = json.load(fh)
        except Exception as exc:
            rospy.logwarn("layout metadata unavailable (%s) — using defaults", exc)
            self.stair_up_entry = STAIR_ENTRY
            self.stair_down_entry = STAIR_ENTRY
            self.stair_exit = STAIR_EXIT
            return

        floors = metadata.get("floors", [])
        if floors:
            bounds = floors[0].get("stair_bounds", {})
            if bounds:
                width = float(bounds["x_max"]) - float(bounds["x_min"])
                x_left  = float(bounds["x_min"]) + 0.26 * width
                x_right = float(bounds["x_max"]) - 0.26 * width
                x_exit  = float(bounds["x_max"]) + 0.50 * width
                y_entry = float(bounds["y_min"]) + 0.65

                self.stair_up_entry   = STAIR_ENTRY
                self.stair_down_entry = STAIR_ENTRY
                self.stair_exit       = (x_exit,  y_entry + 0.50)
                rospy.loginfo("stair geometry loaded: up_entry=%s down_entry=%s exit=%s",
                              self.stair_up_entry, self.stair_down_entry, self.stair_exit)
                return

        self.stair_up_entry = STAIR_ENTRY
        self.stair_down_entry = STAIR_ENTRY
        self.stair_exit = STAIR_EXIT

    # ------------------------------------------------------------------
    # Odometry → floor detection
    # ------------------------------------------------------------------

    def _odom_cb(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        from tf.transformations import euler_from_quaternion
        yaw = euler_from_quaternion((ori.x, ori.y, ori.z, ori.w))[2]
        self.pose = (float(pos.x), float(pos.y), float(pos.z), float(yaw))
        if self.home_z is None:
            self.home_z = float(pos.z)
        self.current_floor = self._floor_from_z(float(pos.z))

    def _floor_from_z(self, z):
        if self.home_z is None or self.floor_height <= 0:
            return 0
        rel = (z - self.home_z) / self.floor_height
        return clamp(int(round(rel)), 0, self.floor_count - 1)

    # ------------------------------------------------------------------
    # Costmap helpers (from auto_room_explorer)
    # ------------------------------------------------------------------

    def _costmap_cb(self, msg):
        self._costmap = msg
        self._costmap_info = msg.info

    def _cell_cost(self, mx, my):
        if self._costmap is None:
            return 255
        if not (0 <= mx < self._costmap_info.width):
            return 255
        if not (0 <= my < self._costmap_info.height):
            return 255
        return self._costmap.data[my * self._costmap_info.width + mx]

    def _is_area_clear(self, x, y):
        if self._costmap is None:
            return True
        info = self._costmap_info
        mx = int((x - info.origin.position.x) / info.resolution)
        my = int((y - info.origin.position.y) / info.resolution)
        cr = max(1, int(0.3 / info.resolution))
        for dx in range(-cr, cr + 1):
            for dy in range(-cr, cr + 1):
                if self._cell_cost(mx + dx, my + dy) >= 100:
                    return False
        return True

    def _is_in_inflation(self, x, y):
        """Check if goal position is within inflation zone (cost > 0).

        If a new obstacle's inflation covers the goal, skip immediately.
        """
        if self._costmap is None:
            return False
        info = self._costmap_info
        mx = int((x - info.origin.position.x) / info.resolution)
        my = int((y - info.origin.position.y) / info.resolution)
        if not (0 <= mx < info.width and 0 <= my < info.height):
            return False
        cr = max(1, int(0.3 / info.resolution))
        for dx in range(-cr, cr + 1):
            for dy in range(-cr, cr + 1):
                cost = self._cell_cost(mx + dx, my + dy)
                if 0 < cost < 100:
                    return True
        return False

    def _world_coord(self, mx, my):
        info = self._costmap_info
        wx = info.origin.position.x + (mx + 0.5) * info.resolution
        wy = info.origin.position.y + (my + 0.5) * info.resolution
        return wx, wy

    def _adjust_goal(self, x, y, exclude=None):
        if self._costmap is None:
            return x, y
        info = self._costmap_info
        mx = int((x - info.origin.position.x) / info.resolution)
        my = int((y - info.origin.position.y) / info.resolution)
        if not (0 <= mx < info.width and 0 <= my < info.height):
            return x, y
        if exclude is None:
            exclude = set()
        if (mx, my) not in exclude and self._is_area_clear(x, y):
            return x, y
        max_r = int(self.search_radius / info.resolution)
        for r in range(1, max_r + 1):
            for dx in range(-r, r + 1):
                for dy in (-r, r):
                    nx, ny = mx + dx, my + dy
                    if (nx, ny) in exclude:
                        continue
                    if not (0 <= nx < info.width and 0 <= ny < info.height):
                        continue
                    wx, wy = self._world_coord(nx, ny)
                    if self._is_area_clear(wx, wy):
                        rospy.loginfo("  → shifted to (%.2f, %.2f)  r=%d", wx, wy, r)
                        return wx, wy
        rospy.logwarn("No clear area within %.1f m — using original", self.search_radius)
        return x, y

    # ------------------------------------------------------------------
    # Move-base goal
    # ------------------------------------------------------------------

    def _make_goal(self, x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation = Quaternion(*q)
        return goal

    def send_goal(self, x, y, yaw, label="", max_retries=3):
        """Send one waypoint to move_base, block until result."""
        self._sent += 1
        prefix = "  %s: " % label if label else ""
        excluded = set()
        for attempt in range(max_retries):
            if rospy.is_shutdown():
                return False
            ax, ay = self._adjust_goal(x, y, exclude=excluded)
            if self._is_in_inflation(ax, ay):
                rospy.logwarn("%sGOAL %02d in inflation zone — skipping",
                              prefix, self._sent)
                return False
            info = self._costmap_info
            if info is not None:
                mx = int((ax - info.origin.position.x) / info.resolution)
                my = int((ay - info.origin.position.y) / info.resolution)
                excluded.add((mx, my))
            goal = self._make_goal(ax, ay, yaw)
            tag = " [SHIFTED]" if (ax != x or ay != y) else ""
            rospy.loginfo("%sGOAL %02d → (%.2f, %.2f, yaw=%.2f)%s",
                          prefix, self._sent, ax, ay, yaw, tag)
            self.ac.send_goal(goal)
            finished = self.ac.wait_for_result(rospy.Duration(self.goal_timeout))
            if not finished:
                rospy.logwarn("%sGOAL %02d TIMED OUT", prefix, self._sent)
                self.ac.cancel_goal()
                return False  # skip to next waypoint immediately
            state = self.ac.get_state()
            if state == GoalStatus.SUCCEEDED:
                self._ok += 1
                rospy.loginfo("%sGOAL %02d OK", prefix, self._sent)
                return True
            names = {GoalStatus.ABORTED: "ABORTED", GoalStatus.REJECTED: "REJECTED",
                     GoalStatus.PREEMPTED: "PREEMPTED", GoalStatus.LOST: "LOST"}
            rospy.logwarn("%sGOAL %02d %s", prefix, self._sent, names.get(state, str(state)))
            if attempt < max_retries - 1:
                rospy.loginfo("  retrying with shifted goal...")
        rospy.logerr("%sGOAL %02d FAILED after %d attempts", prefix, self._sent, max_retries)
        return False

    # ------------------------------------------------------------------
    # Stair climbing
    # ------------------------------------------------------------------

    def _stair_result_cb(self, msg):
        self._stair_result = msg.data
        self._stair_done = True

    def _stair_status_cb(self, msg):
        rospy.loginfo("  stair: %s", msg.data)

    def climb_stairs(self, direction):
        """Trigger stair_climber and block until completion.

        direction: "up" or "down"
        Returns True on success.
        """
        # Cancel any active move_base goal before stair climb
        self.ac.cancel_all_goals()
        rospy.sleep(0.5)  # let move_base settle

        self._stair_result = None
        self._stair_done = False

        target_floor = (self.current_floor + 1 if direction == "up"
                        else self.current_floor - 1)
        rospy.loginfo("=== STAIR CLIMB %s: floor %d → %d ===",
                      direction.upper(), self.current_floor, target_floor)

        self._stair_goal_pub.publish(String(direction))

        # Wait for completion
        timeout = rospy.Time.now() + rospy.Duration(STAIR_CLIMB_TIMEOUT)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self._stair_done:
            if rospy.Time.now() > timeout:
                rospy.logerr("stair climb TIMED OUT after %.0f s", STAIR_CLIMB_TIMEOUT)
                return False
            rate.sleep()

        # Let robot settle on new floor
        rospy.sleep(POST_STAIR_SLEEP)

        result = self._stair_result or "unknown"
        success = result.startswith("success")
        rospy.loginfo("=== STAIR CLIMB %s: %s ===", direction.upper(), result)

        if success:
            # Update floor tracking
            for _ in range(10):
                rospy.sleep(0.1)
                if self.current_floor == target_floor:
                    break
            rospy.loginfo("  current floor after climb: %d", self.current_floor)

        return success

    def reinit_amcl(self):
        """Reinitialize AMCL at the current odometry pose after stair climbing.

        During 3D stair motion, AMCL particles diverge because the stair
        pointcloud does not match the 2D static map.  Publishing a fresh
        initial pose with large covariance lets AMCL re-converge.
        """
        if self.pose is None:
            rospy.logwarn("No odometry pose available for AMCL reinit")
            return

        x, y, z, yaw = self.pose
        rospy.loginfo("Reinitializing AMCL at (%.2f, %.2f, yaw=%.2f, floor=%d)",
                      x, y, yaw, self.current_floor)

        init_pose = PoseWithCovarianceStamped()
        init_pose.header.frame_id = "map"
        init_pose.header.stamp = rospy.Time.now()
        init_pose.pose.pose.position.x = x
        init_pose.pose.pose.position.y = y
        init_pose.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, yaw)
        init_pose.pose.pose.orientation = Quaternion(*q)

        # Large covariance so AMCL re-converges from laser scans
        init_pose.pose.covariance = [0.0] * 36
        init_pose.pose.covariance[0] = 0.5     # x  std ~0.71 m
        init_pose.pose.covariance[7] = 0.5     # y  std ~0.71 m
        init_pose.pose.covariance[35] = 0.3    # yaw std ~0.55 rad

        # Publish several times to help AMCL settle
        for i in range(5):
            init_pose.header.stamp = rospy.Time.now()
            self._initpose_pub.publish(init_pose)
            rospy.sleep(0.2)

        # Give AMCL time to converge
        rospy.sleep(1.5)

        # Also clear costmaps to remove stale obstacle data from stairs
        try:
            rospy.wait_for_service("/move_base/clear_costmaps", 3.0)
            clear = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
            clear()
            rospy.loginfo("Costmaps cleared")
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logwarn("Could not clear costmaps: %s", e)

        rospy.loginfo("AMCL reinitialization complete")

    # ------------------------------------------------------------------
    # Navigation helpers
    # ------------------------------------------------------------------

    def navigate_to_stair_entry(self, direction):
        """Navigate directly to the stair entry position."""
        entry = self.stair_up_entry if direction == "up" else self.stair_down_entry
        rospy.loginfo("navigating to stair %s entry: (%.2f, %.2f)",
                      direction, entry[0], entry[1])
        return self.send_goal(entry[0], entry[1], entry[2],
                             label="stair_approach_%s" % direction)

    # ------------------------------------------------------------------
    # Floor execution
    # ------------------------------------------------------------------

    def run_floor(self, floor_index):
        """Execute all waypoints for one floor."""
        rospy.loginfo("")
        rospy.loginfo("########## FLOOR %d — %d waypoints ##########",
                      floor_index, len(FLOOR_WAYPOINTS))

        for i, (x, y, yaw) in enumerate(FLOOR_WAYPOINTS):
            if rospy.is_shutdown():
                return False
            label = "F%d-wp%02d" % (floor_index, i + 1)
            rospy.loginfo("—" * 40 + "  %s", label)
            if not self.send_goal(x, y, yaw, label=label):
                rospy.logwarn("waypoint %d on floor %d failed — continuing", i + 1, floor_index)
                # continue with next waypoint even if one fails

        self._floors_completed.add(floor_index)
        rospy.loginfo("########## FLOOR %d COMPLETE ##########", floor_index)
        return True

    # ------------------------------------------------------------------
    # Main multi-floor loop
    # ------------------------------------------------------------------

    def run(self):
        rospy.loginfo("=== MULTI-FLOOR EXPLORER — %d floors ===", self.floor_count)
        rospy.loginfo("waypoints per floor: %d", len(FLOOR_WAYPOINTS))
        rospy.loginfo("stair up entry:   (%.2f, %.2f)", *self.stair_up_entry[:2])
        rospy.loginfo("stair down entry: (%.2f, %.2f)", *self.stair_down_entry[:2])

        # Wait for initial pose
        while self.pose is None and not rospy.is_shutdown():
            rospy.sleep(0.2)
        rospy.loginfo("initial pose: (%.2f, %.2f, z=%.2f) floor=%d",
                      self.pose[0], self.pose[1], self.pose[2], self.current_floor)

        # ---- Floor 0: ground-floor exploration ----
        self.run_floor(0)

        # ---- Floor 1: climb up then explore ----
        if self.floor_count >= 2 and not rospy.is_shutdown():
            rospy.loginfo("")
            rospy.loginfo("========== TRANSITION: floor 0 → 1 ==========")
            self.navigate_to_stair_entry("up")
            if self.climb_stairs("up"):
                self.reinit_amcl()  # fix AMCL after 3D stair motion
                # Navigate from stair exit to first corridor waypoint
                self.send_goal(
                    FLOOR_START_WP[0], FLOOR_START_WP[1], FLOOR_START_WP[2],
                    label="F1-entry"
                )
                self.run_floor(1)
            else:
                rospy.logerr("stair climb 0→1 failed — aborting upper floors")

        # ---- Floor 2: climb up then explore ----
        if self.floor_count >= 3 and not rospy.is_shutdown() and 1 in self._floors_completed:
            rospy.loginfo("")
            rospy.loginfo("========== TRANSITION: floor 1 → 2 ==========")
            self.navigate_to_stair_entry("up")
            if self.climb_stairs("up"):
                self.reinit_amcl()  # fix AMCL after 3D stair motion
                self.send_goal(
                    FLOOR_START_WP[0], FLOOR_START_WP[1], FLOOR_START_WP[2],
                    label="F2-entry"
                )
                self.run_floor(2)
            else:
                rospy.logerr("stair climb 1→2 failed")

        # ---- Return: descend to ground floor ----
        if not rospy.is_shutdown():
            rospy.loginfo("")
            rospy.loginfo("========== RETURN: descending to ground ==========")

            # Descend from current floor to floor 0
            while self.current_floor > 0 and not rospy.is_shutdown():
                rospy.loginfo("  descending: floor %d → %d", self.current_floor, self.current_floor - 1)
                self.navigate_to_stair_entry("down")
                if not self.climb_stairs("down"):
                    rospy.logerr("stair climb down failed at floor %d", self.current_floor)
                    break
                self.reinit_amcl()  # fix AMCL after 3D stair motion

            # Final return to home
            if self.current_floor == 0:
                rospy.loginfo("navigating to home: (%.2f, %.2f)", HOME[0], HOME[1])
                self.send_goal(HOME[0], HOME[1], HOME[2], label="return_home")

        rospy.loginfo("=== MULTI-FLOOR EXPLORER DONE  %d/%d goals reached ===",
                      self._ok, self._sent)
        rospy.loginfo("floors completed: %s", sorted(self._floors_completed))


if __name__ == "__main__":
    try:
        MultiFloorExplorer().run()
    except rospy.ROSInterruptException:
        pass
