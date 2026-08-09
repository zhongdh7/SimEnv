#!/usr/bin/env python3
"""
TEST: Skip floor-0, climb to floor-1, then run the waypoint sequence on floor-1.

Purpose: verify that the floor-0 waypoint algorithm works on floor-1 (二楼).

Flow:  start on floor-0 → navigate to stair entry → climb up
    → floor-1 corridor entrance → run 22 waypoints → done
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


# ---- same 22 waypoints (XY identical for all floors) ----
FLOOR_WAYPOINTS = [
    ( 0.256, 14.641,  1.565),
    (-1.941, 14.722,  3.118),
    (-7.649, 20.250, -0.636),
    (-7.785,  9.343,  0.594),
    (-1.923, 14.477,  0.089),
    ( 2.387, 14.597,  0.000),
    ( 7.920, 19.999, -2.543),
    ( 8.142,  9.207,  2.550),
    ( 2.624, 14.554,  3.130),
    ( 0.290, 14.641, -3.121),
    ( 0.511, 28.162,  1.572),
    (-2.365, 28.269,  3.120),
    (-7.255, 33.750, -0.652),
    (-7.291, 22.966,  0.623),
    (-1.978, 28.193,  0.049),
    ( 2.784, 28.069,  0.000),
    ( 8.394, 33.405, -2.587),
    ( 8.596, 22.847,  2.582),
    ( 3.102, 27.996,  3.114),
    ( 0.440, 28.162, -3.131),
    ( 0.050,  1.997, -1.489),
    (-2.844,  1.615,  2.937),
]

# ---- stair geometry ----
ENTRANCE_WP      = ( 0.000,  2.000,  1.571)   # just inside main entrance
STAIR_UP_ENTRY   = (-2.844,  1.615,  2.937)
FLOOR_START_WP   = ( 0.256, 14.641,  1.565)

# ---- tunable ----
GOAL_TIMEOUT      = 120.0
GOAL_SEARCH_RADIUS = 2.0
STAIR_CLIMB_TIMEOUT = 300.0


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class TestFloor2Explorer:
    """Skip floor-0 → climb stairs → explore floor-1."""

    def __init__(self):
        rospy.init_node("test_floor2_explorer")

        # --- move_base ---
        rospy.loginfo("waiting for move_base ...")
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        if not self.ac.wait_for_server(rospy.Duration(30)):
            raise RuntimeError("move_base not available")
        rospy.loginfo("move_base connected")

        # --- costmap ---
        self._costmap = None
        self._costmap_info = None
        rospy.Subscriber("/move_base/global_costmap/costmap",
                         OccupancyGrid, self._costmap_cb)
        deadline = rospy.Time.now() + rospy.Duration(15.0)
        while self._costmap is None and rospy.Time.now() < deadline:
            rospy.sleep(0.2)
        if self._costmap is not None:
            rospy.loginfo("costmap ready (%dx%d %.3f m/pix)",
                          self._costmap_info.width,
                          self._costmap_info.height,
                          self._costmap_info.resolution)

        # --- odometry ---
        self.pose = None
        self.home_z = None
        self.current_floor = 0
        rospy.Subscriber("/Odometry_gazebo", Odometry, self._odom_cb)

        # --- stair climber ---
        self._stair_result = None
        self._stair_done = False
        rospy.Subscriber("/stair_climb/result", String, self._stair_result_cb)
        rospy.Subscriber("/stair_climb/status", String, self._stair_status_cb)
        self._stair_goal_pub = rospy.Publisher("/stair_climb/goal", String, queue_size=1)
        self._initpose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)

        self._sent = 0
        self._ok = 0

    # ------------------------------------------------------------------
    # Odometry
    # ------------------------------------------------------------------

    def _odom_cb(self, msg):
        from tf.transformations import euler_from_quaternion
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = euler_from_quaternion((ori.x, ori.y, ori.z, ori.w))[2]
        self.pose = (float(pos.x), float(pos.y), float(pos.z), float(yaw))
        if self.home_z is None:
            self.home_z = float(pos.z)
        if self.home_z is not None:
            rel = (float(pos.z) - self.home_z) / 2.6
            self.current_floor = clamp(int(round(rel)), 0, 2)

    # ------------------------------------------------------------------
    # Costmap
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
        max_r = int(GOAL_SEARCH_RADIUS / info.resolution)
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
        rospy.logwarn("no clear area within %.1f m — using original", GOAL_SEARCH_RADIUS)
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
        self._sent += 1
        excluded = set()
        for attempt in range(max_retries):
            if rospy.is_shutdown():
                return False
            ax, ay = self._adjust_goal(x, y, exclude=excluded)
            info = self._costmap_info
            if info is not None:
                mx = int((ax - info.origin.position.x) / info.resolution)
                my = int((ay - info.origin.position.y) / info.resolution)
                excluded.add((mx, my))
            goal = self._make_goal(ax, ay, yaw)
            prefix = "  %s: " % label if label else ""
            rospy.loginfo("%sGOAL %02d → (%.2f, %.2f, yaw=%.2f)",
                          prefix, self._sent, ax, ay, yaw)
            self.ac.send_goal(goal)
            finished = self.ac.wait_for_result(rospy.Duration(GOAL_TIMEOUT))
            if not finished:
                rospy.logwarn("%sGOAL %02d TIMED OUT", prefix, self._sent)
                self.ac.cancel_goal()
                continue
            state = self.ac.get_state()
            if state == GoalStatus.SUCCEEDED:
                self._ok += 1
                rospy.loginfo("%sGOAL %02d OK", prefix, self._sent)
                return True
            names = {GoalStatus.ABORTED: "ABORTED", GoalStatus.REJECTED: "REJECTED",
                     GoalStatus.PREEMPTED: "PREEMPTED", GoalStatus.LOST: "LOST"}
            rospy.logwarn("%sGOAL %02d %s", prefix, self._sent, names.get(state, str(state)))
        rospy.logerr("%sGOAL %02d FAILED", prefix, self._sent)
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
        self.ac.cancel_all_goals()
        rospy.sleep(0.5)

        self._stair_result = None
        self._stair_done = False
        target = self.current_floor + (1 if direction == "up" else -1)
        rospy.loginfo("=== STAIR CLIMB %s: floor %d → %d ===",
                      direction.upper(), self.current_floor, target)
        self._stair_goal_pub.publish(String(direction))

        timeout = rospy.Time.now() + rospy.Duration(STAIR_CLIMB_TIMEOUT)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self._stair_done:
            if rospy.Time.now() > timeout:
                rospy.logerr("stair climb TIMED OUT")
                return False
            rate.sleep()

        rospy.sleep(2.0)  # settle
        result = self._stair_result or "unknown"
        success = result.startswith("success")
        rospy.loginfo("=== STAIR CLIMB %s: %s ===", direction.upper(), result)
        return success

    # ------------------------------------------------------------------
    # AMCL reinitialization
    # ------------------------------------------------------------------

    def reinit_amcl(self):
        """Reinitialize AMCL at the current odometry pose after stair climbing.

        During 3D stair motion, AMCL particles diverge because the stair
        pointcloud does not match the 2D static map.  Publishing a fresh
        initial pose with large covariance lets AMCL re-converge using the
        laser scans on the new floor.
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
    # Main
    # ------------------------------------------------------------------

    def run(self):
        rospy.loginfo("=== TEST: skip floor-0 → stairs up → explore floor-1 ===")

        # Wait for initial pose
        while self.pose is None and not rospy.is_shutdown():
            rospy.sleep(0.2)
        rospy.loginfo("initial pose: (%.2f, %.2f, z=%.2f) floor=%d",
                      self.pose[0], self.pose[1], self.pose[2], self.current_floor)

        # ---- Step 1: enter building through main entrance ----
        rospy.loginfo("")
        rospy.loginfo("========== STEP 1: enter building ==========")
        self.send_goal(ENTRANCE_WP[0], ENTRANCE_WP[1], ENTRANCE_WP[2],
                       label="enter_gate")

        # ---- Step 2: navigate to stair entry ----
        rospy.loginfo("")
        rospy.loginfo("========== STEP 2: approach stair entry ==========")
        self.send_goal(STAIR_UP_ENTRY[0], STAIR_UP_ENTRY[1], STAIR_UP_ENTRY[2],
                       label="stair_approach")

        # ---- Step 3: climb up to floor 1 ----
        rospy.loginfo("")
        rospy.loginfo("========== STEP 3: climb stairs to floor 1 ==========")
        if not self.climb_stairs("up"):
            rospy.logerr("stair climb failed — abort")
            return
        rospy.loginfo("current floor after climb: %d (z=%.3f)",
                      self.current_floor, self.pose[2] if self.pose else -999)

        # ---- Step 3.5: reinitialize AMCL on new floor ----
        # Essential: AMCL loses localization during 3D stair motion.
        # Without this the local costmap freezes after climbing.
        rospy.loginfo("")
        rospy.loginfo("========== STEP 3.5: reinitialize AMCL on floor 1 ==========")
        self.reinit_amcl()

        # ---- Step 4: enter corridor on floor 1 ----
        rospy.loginfo("")
        rospy.loginfo("========== STEP 4: enter floor-1 corridor ==========")
        self.send_goal(FLOOR_START_WP[0], FLOOR_START_WP[1], FLOOR_START_WP[2],
                       label="F1-entry")

        # ---- Step 5: run floor-0 waypoints on floor 1 ----
        rospy.loginfo("")
        rospy.loginfo("========== STEP 5: explore floor 1 (22 waypoints) ==========")
        for i, (x, y, yaw) in enumerate(FLOOR_WAYPOINTS):
            if rospy.is_shutdown():
                return
            label = "F1-wp%02d" % (i + 1)
            rospy.loginfo("—" * 40 + "  %s", label)
            self.send_goal(x, y, yaw, label=label)

        rospy.loginfo("")
        rospy.loginfo("=== TEST DONE  %d/%d goals reached ===", self._ok, self._sent)


if __name__ == "__main__":
    try:
        TestFloor2Explorer().run()
    except rospy.ROSInterruptException:
        pass
