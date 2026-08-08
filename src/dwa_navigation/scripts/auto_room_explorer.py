#!/usr/bin/env python3
"""
Auto Room Explorer — sends navigation goals in a fixed sequence.

Each entry is (x, y, yaw).  Goals are sent in order; if a target lands
inside an obstacle the script shifts it to the nearest free cell
(checked against /move_base/global_costmap/costmap).

Positions from previous RViz recording; yaw angles from latest recording.
"""

import math
import rospy
import actionlib
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler


# ================================================================
# WAYPOINT SEQUENCE — (x, y, yaw) in map frame
# ================================================================
WAYPOINTS = [
    # === Phase 1: Enter building → corridor midpoint ===
    ( 0.230, 14.850,  1.562),
    (-1.818, 14.614,  3.115),

    # === Phase 2: Left rooms (west wing, y≈9–21) ===
    (-2.000, 20.100, -2.262),
    (-7.850, 20.100, -0.936),
    (-8.402,  8.833,  0.811),
    (-2.200,  9.100,  2.044),

    # === Phase 3: Back to corridor ===
    (-1.820, 14.661,  1.525),
    ( 0.315, 14.644,  0.021),

    # === Phase 4: Right rooms (east wing, y≈9–21) ===
    ( 2.362, 14.672,  0.037),
    ( 2.285, 20.698, -0.813),
    ( 8.511, 20.425, -2.232),
    ( 8.537,  8.861,  2.198),
    ( 2.396,  8.699,  0.866),

    # === Phase 5: Back to corridor ===
    ( 2.250, 14.432,  1.482),
    ( 0.181, 14.659, -3.127),

    # === Phase 6: Transit to far corridor (y≈28) ===
    ( 0.510, 27.956,  1.502),

    # === Phase 7: Far-left rooms (west wing, y≈22–34) ===
    (-1.717, 28.243,  3.110),
    (-1.825, 34.148, -2.137),
    (-7.612, 34.148, -1.001),
    (-7.821, 22.548,  0.940),
    (-1.770, 22.287,  2.107),

    # === Phase 8: Back to far corridor ===
    (-1.506, 28.125,  1.584),
    ( 0.513, 28.217,  0.122),

    # === Phase 9: Far-right rooms (east wing, y≈22–34) ===
    ( 2.476, 28.150,  0.046),
    ( 2.688, 34.064, -0.939),
    ( 8.794, 34.055, -2.089),
    ( 8.848, 22.127,  2.099),
    ( 2.477, 22.236,  0.791),

    # === Phase 10: Return to entrance ===
    (-0.368,  1.945, -1.563),
    (-2.366,  1.834, -3.125),
]


# ================================================================
# TUNABLE
# ================================================================
GOAL_TIMEOUT = 120.0
GOAL_SEARCH_RADIUS = 2.0


class WaypointFollower:
    """Send each waypoint to move_base in order."""

    def __init__(self):
        rospy.init_node("auto_room_explorer")

        rospy.loginfo("Waiting for move_base action server...")
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        if not self.ac.wait_for_server(rospy.Duration(30)):
            raise RuntimeError("move_base not available")
        rospy.loginfo("move_base connected")

        # --- Global costmap for goal validation ---
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

        self.goal_timeout = rospy.get_param("~goal_timeout", GOAL_TIMEOUT)
        self.search_radius = rospy.get_param("~goal_search_radius",
                                             GOAL_SEARCH_RADIUS)
        self._sent = 0
        self._ok = 0

    # ------------------------------------------------------------------
    # Costmap helpers
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
        my  = int((y - info.origin.position.y) / info.resolution)
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
                        rospy.loginfo("  → shifted to (%.2f, %.2f)  r=%d",
                                      wx, wy, r)
                        return wx, wy
        rospy.logwarn("No clear area within %.1f m — using original",
                      self.search_radius)
        return x, y

    # ------------------------------------------------------------------
    # Navigation
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

    def send_goal(self, x, y, yaw, max_retries=3):
        self._sent += 1
        excluded = set()
        for attempt in range(max_retries):
            ax, ay = self._adjust_goal(x, y, exclude=excluded)
            info = self._costmap_info
            if info is not None:
                mx = int((ax - info.origin.position.x) / info.resolution)
                my = int((ay - info.origin.position.y) / info.resolution)
                excluded.add((mx, my))
            goal = self._make_goal(ax, ay, yaw)
            tag = ""
            if ax != x or ay != y:
                tag = " [SHIFTED from (%.2f,%.2f)]" % (x, y)
            atag = " (retry %d)" % attempt if attempt > 0 else ""
            rospy.loginfo("GOAL %02d%s → (%.2f, %.2f, yaw=%.2f)%s",
                          self._sent, atag, ax, ay, yaw, tag)
            self.ac.send_goal(goal)
            finished = self.ac.wait_for_result(
                rospy.Duration(self.goal_timeout))
            if not finished:
                rospy.logwarn("GOAL %02d TIMED OUT", self._sent)
                self.ac.cancel_goal()
                if attempt < max_retries - 1:
                    rospy.loginfo("  retrying with shifted goal...")
                continue
            state = self.ac.get_state()
            if state == GoalStatus.SUCCEEDED:
                self._ok += 1
                rospy.loginfo("GOAL %02d OK", self._sent)
                return True
            names = {GoalStatus.ABORTED: "ABORTED",
                     GoalStatus.REJECTED: "REJECTED",
                     GoalStatus.PREEMPTED: "PREEMPTED",
                     GoalStatus.LOST: "LOST"}
            rospy.logwarn("GOAL %02d %s", self._sent,
                          names.get(state, str(state)))
            if attempt < max_retries - 1:
                rospy.loginfo("  retrying with shifted goal...")
        rospy.logerr("GOAL %02d FAILED after %d attempts",
                     self._sent, max_retries)
        return False

    # ------------------------------------------------------------------
    # Main
    # ------------------------------------------------------------------

    def run(self):
        rospy.loginfo("=== AUTO EXPLORER — %d waypoints ===", len(WAYPOINTS))
        for i, (x, y, yaw) in enumerate(WAYPOINTS):
            if rospy.is_shutdown():
                break
            rospy.loginfo("—" * 30 + "  wp %d", i + 1)
            self.send_goal(x, y, yaw)
        rospy.loginfo("=== DONE  %d/%d reached ===", self._ok, self._sent)


if __name__ == "__main__":
    try:
        WaypointFollower().run()
    except rospy.ROSInterruptException:
        pass
