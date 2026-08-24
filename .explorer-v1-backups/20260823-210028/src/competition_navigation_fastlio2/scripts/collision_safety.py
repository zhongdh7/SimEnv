#!/usr/bin/env python3
"""Collision-safety layer (independent node -- does NOT modify the exploration logic).

The navigation controller is a pure waypoint tracker with no local obstacle
avoidance: it computes ``linear.x = linear_gain * body_x`` toward the next
waypoint and publishes it directly on ``/cmd_vel``.  When the planner routes a
path through a wall (or the robot overshoots a waypoint into one), the
controller drives straight into it -- the wall-hugging / fall scenario.

This node sits between the navigation node and the robot's servo so the robot
never pushes into a mapped obstacle:

    competition_navigation_node  --/cmd_vel_raw-->  collision_safety  --/cmd_vel-->  unitree_gazebo_servo

It reads the navigation node's own published 2-D occupancy map (already
flattened and self-filtered by the nav), plus the robot pose, and zeroes the
forward/lateral velocity when an occupied cell lies inside a small safety box
ahead of the direction of travel.  The yaw-rate command is left intact so the
robot can still turn away.  A stale command (nav died) also drops to zero.

Stair handling: the 2-D map is a flattened projection, so while the robot is
actually climbing a flight the next riser/step ahead is (correctly) raytraced
as an occupied cell at body height.  Blocking on that would freeze the robot
halfway up.  The stair flight is a deterministic waypoint route with lateral
motion clamped to ``stair_lateral_limit``, so it is safe to pass the command
through unchanged there.  This node therefore watches the navigation node's
``/competition_navigation/status`` topic and disables the safety box while
``mode`` is ``stairs_up`` or ``stairs_down`` (the approach modes
``go_stairs_up``/``go_stairs_down`` still run on flat ground and keep blocking).

Recovery: when forward motion is blocked by a real obstacle, this node backs
the robot off a short distance (straight reverse) and then holds still, giving
the navigation node's own stall detection time to re-plan around the obstacle.
This is a self-contained recovery manoeuvre -- it never modifies the exploration
logic; it only rewrites the outgoing cmd_vel during the back-off.

Tunables (all via private params, defaults chosen for the A1 in SimEnv):
  safety_dist        -- stop when an obstacle is within this many metres ahead
  safety_half_width  -- half-width of the safety box (kept < a doorway's half
                        width so passing through doors does not false-trigger)
  min_check_dist     -- ignore cells closer than this (the robot's own body)
  status_topic       -- nav status (String) used to detect an active stair flight
  recover_enable     -- master switch for the reverse-and-hold recovery
  reverse_speed      -- back-off speed (m/s, sign ignored, always reverses)
  reverse_dist       -- how far to back off before holding
  reverse_max_time   -- cap on the back-off duration even if reverse_dist unmet
  recover_hold_time  -- how long to hold still after backing off
  recover_confirm_time -- block must persist this long before recovery starts
  recover_cooldown   -- min time between recoveries; kept longer than the nav's
                        room_entry_stall_timeout so the re-plan fires during the
                        hold instead of being reset by another back-off
"""

import math
import re

import rospy
import tf.transformations as tft
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud
from std_msgs.msg import String

OCCUPIED = 100
FREE = 0
# How far past an occupied cell to look for FREE space when deciding whether it
# is a solid wall (occluded behind) or a floating phantom obstacle (free behind).
PHANTOM_LOOKAHEAD = 1.5


class CollisionSafety(object):
    def __init__(self):
        self.cmd_in = rospy.get_param("~cmd_in", "/cmd_vel_raw")
        self.cmd_out = rospy.get_param("~cmd_out", "/cmd_vel")
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry_fastlio")
        self.map_topic = rospy.get_param("~map_topic", "/competition_navigation/map")
        self.status_topic = rospy.get_param(
            "~status_topic", "/competition_navigation/status"
        )
        self.rate = float(rospy.get_param("~rate", 20.0))
        self.stale_timeout = rospy.Duration(
            float(rospy.get_param("~stale_timeout", 0.6))
        )

        self.safety_dist = float(rospy.get_param("~safety_dist", 0.6))
        self.safety_half_width = float(rospy.get_param("~safety_half_width", 0.35))
        self.min_check_dist = float(rospy.get_param("~min_check_dist", 0.25))

        # Reverse-and-hold recovery: back off when a real obstacle blocks forward
        # motion, then hold so the nav node's stall detection re-plans.
        self.recover_enable = bool(rospy.get_param("~recover_enable", True))
        self.reverse_speed = -abs(float(rospy.get_param("~reverse_speed", 0.30)))
        self.reverse_dist = float(rospy.get_param("~reverse_dist", 0.45))
        self.reverse_max_time = float(rospy.get_param("~reverse_max_time", 4.0))
        self.recover_hold_time = float(rospy.get_param("~recover_hold_time", 1.5))
        self.recover_confirm_time = float(rospy.get_param("~recover_confirm_time", 0.8))
        # Kept longer than the nav node's room_entry_stall_timeout (10 s) so the
        # re-plan fires during the cooldown hold instead of being reset by
        # another back-off before the stall clock can expire.
        self.recover_cooldown = float(rospy.get_param("~recover_cooldown", 15.0))
        # Turn-in-place escape: when the robot is wedged (forward blocked AND
        # reverse also blocked -- e.g. a corridor-side alcove with the building
        # boundary wall ahead and a wall behind), reverse-and-hold would freeze
        # it forever.  After a blocked reverse, rotate in place for
        # turn_duration at turn_yaw_rate so the nav node can re-plan along a
        # different heading.  Measured trap on floor-0: robot at x=-3.83 in the
        # left-wall channel with the x=-5 boundary wall ahead and obstacles
        # behind (map lines OOOOO... ahead, OO... behind).
        self.turn_recovery = bool(rospy.get_param("~turn_recovery", True))
        self.turn_yaw_rate = abs(float(rospy.get_param("~turn_yaw_rate", 0.5)))
        self.turn_duration = float(rospy.get_param("~turn_duration", 2.5))

        # Direct laser-based collision detection (primary, map-independent).
        # The 2-D occupancy map misses walls under FAST-LIO2 drift (wall cells
        # left UNKNOWN/FREE at the collision spot), so map-only braking let
        # the robot drive into real walls.  The live /scan sees the true
        # obstacle regardless of the map: transform the cloud into the map
        # frame, then brake if any point lies inside a forward safety cone
        # ahead of travel.  Ghost obstacles (map artefacts) are NOT seen by the
        # laser, so they never false-trigger.
        self.laser_topic = rospy.get_param("~laser_topic", "/scan")
        self.laser_safety_dist = float(rospy.get_param("~laser_safety_dist", 0.55))
        self.laser_safety_half_width = float(
            rospy.get_param("~laser_safety_half_width", 0.35)
        )
        # Ignore points closer than this (the robot's own body/legs).
        self.laser_safety_min_dist = float(
            rospy.get_param("~laser_safety_min_dist", 0.25)
        )
        # Height window around the robot's z in which laser points count.
        self.laser_max_height_offset = float(
            rospy.get_param("~laser_max_height_offset", 0.50)
        )
        self._laser_pts = []      # points in the map frame (x, y, z)
        self._laser_stamp = None
        self._tf = None

        self.map = None
        self.pose = None  # (x, y, z, yaw) in the map frame
        self.in_stairs = False
        self.last_cmd = Twist()
        self.last_cmd_stamp = None
        self.recover_phase = None  # None | "reverse" | "hold"
        self.recover_start_pose = None
        self.recover_start_time = None
        self.blocked_since = None
        self.last_recover_end = rospy.Time(0)

        self.pub = rospy.Publisher(self.cmd_out, Twist, queue_size=5)
        rospy.Subscriber(self.cmd_in, Twist, self._cmd_cb)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb)
        rospy.Subscriber(self.map_topic, OccupancyGrid, self._map_cb)
        rospy.Subscriber(self.status_topic, String, self._status_cb)
        try:
            import tf as _tf
            self._tf = _tf.TransformListener(
                cache_time=rospy.Duration(10.0)
            )
        except Exception as exc:  # pragma: no cover
            rospy.logwarn("collision_safety: tf unavailable (%s)", exc)
            self._tf = None
        rospy.Subscriber(self.laser_topic, PointCloud, self._scan_cb, queue_size=1)
        rospy.loginfo(
            "collision_safety: %s -> %s (dist=%.2fm half-width=%.2fm rate=%.0fHz laser=%s)",
            self.cmd_in, self.cmd_out,
            self.safety_dist, self.safety_half_width, self.rate, self.laser_topic,
        )

    def _scan_cb(self, msg):
        if self._tf is None or self.pose is None:
            return
        try:
            (translation, rotation) = self._tf.lookupTransform(
                "odom", msg.header.frame_id, msg.header.stamp
            )
        except (tf.LookupException, tf.ConnectivityException):
            return
        except tf.ExtrapolationException:
            try:
                (translation, rotation) = self._tf.lookupTransform(
                    "odom", msg.header.frame_id, rospy.Time(0)
                )
            except (tf.LookupException, tf.ConnectivityException):
                return
        matrix = tft.quaternion_matrix(rotation)
        matrix[0:3, 3] = translation
        points = []
        for point in msg.points:
            if not all(math.isfinite(v) for v in
                       (point.x, point.y, point.z)):
                continue
            world = matrix.dot(
                [point.x, point.y, point.z, 1.0]
            )
            points.append((world[0], world[1], world[2]))
        self._laser_pts = points
        self._laser_stamp = msg.header.stamp

    def _laser_blocked(self):
        """True when live laser points lie inside the forward safety cone."""
        if not self._laser_pts or self.pose is None:
            return False
        x, y, z, yaw = self.pose
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        for (px, py, pz) in self._laser_pts:
            # Height gate: only points at or slightly below the trunk (legs
            # excluded) count.  /scan is a 45-deg-pitched 3-D cloud, so the
            # floor ~0.3 m below the robot must NOT be treated as an obstacle
            # (it tripped the laser brake at the spawn point).
            if pz < z - 0.05 or pz > z + self.laser_max_height_offset:
                continue
            dx = px - x
            dy = py - y
            forward = dx * cos_yaw + dy * sin_yaw
            if (forward < self.laser_safety_min_dist or
                    forward > self.laser_safety_dist):
                continue
            lateral = -dx * sin_yaw + dy * cos_yaw
            if abs(lateral) <= self.laser_safety_half_width:
                return True
        return False

    def _map_cb(self, msg):
        self.map = msg

    def _status_cb(self, msg):
        # The nav node publishes a composite status string that always carries
        # "mode=<state>".  Only the actual stair flight (stairs_up/stairs_down)
        # disables the safety box; the approach modes still run on flat ground.
        match = re.search(r"\bmode=(\S+)", msg.data)
        in_stairs = bool(match) and match.group(1) in ("stairs_up", "stairs_down")
        if in_stairs != self.in_stairs:
            self.in_stairs = in_stairs
            rospy.loginfo(
                "collision_safety: stair flight %s -> safety box %s",
                "ACTIVE" if in_stairs else "inactive",
                "DISABLED" if in_stairs else "enabled",
            )

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose = (p.x, p.y, p.z, yaw)

    def _cmd_cb(self, msg):
        self.last_cmd = msg
        self.last_cmd_stamp = rospy.Time.now()

    def _occ(self, x, y):
        m = self.map
        if m is None:
            return None
        res = m.info.resolution
        cx = int(math.floor((x - m.info.origin.position.x) / res))
        cy = int(math.floor((y - m.info.origin.position.y) / res))
        if 0 <= cx < m.info.width and 0 <= cy < m.info.height:
            return m.data[cy * m.info.width + cx]
        return None

    def _free_behind(self, x, y, sign, cos_yaw, sin_yaw, start_d, res):
        """True if FREE space lies beyond the occupied cell(s) along the heading.

        A genuine wall face is occluded: the cells directly behind it are
        UNKNOWN (never raytraced).  A phantom cell/cluster left by 3-D
        flattening floats in open space, so the cells beyond it have been
        raytraced FREE.  Scanning past any contiguous occupied run up to
        PHANTOM_LOOKAHEAD therefore distinguishes the two: FREE beyond means no
        solid wall is in the way, so collision_safety must not block.
        """
        d = start_d + res
        limit = start_d + PHANTOM_LOOKAHEAD
        while d <= limit + 1e-6:
            v = self._occ(x + sign * d * cos_yaw, y + sign * d * sin_yaw)
            if v == FREE:
                return True
            if v != OCCUPIED:
                # UNKNOWN or outside the map -> occluded wall face.
                return False
            d += res
        return False

    def _obstacle_is_isolated(self, x, y, cos_yaw, sin_yaw, d):
        """True only for a SINGLE-cell phantom obstacle.

        A ghost cell left by 3-D flattening / drift is a lone occupied cell in
        open space: its 3x3 neighbourhood (perpendicular AND along travel) has
        no other OCCUPIED cell.  Any cluster -- a genuine wall, a door frame,
        or a few-cell ghost blob -- has OCCUPIED neighbours and is treated as
        solid, so the robot always brakes in front of it (it then backs off,
        turns and the nav re-plans around it).  A direction-agnostic 3x3 check
        is robust to angled approaches, where a lateral-run measurement fails
        (a wall hit at ~45 deg shows only 1-2 occupied cells across the
        heading)."""
        res = self.map.info.resolution if self.map is not None else 0.2
        ox = x + d * cos_yaw
        oy = y + d * sin_yaw
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                if self._occ(ox + dx * res, oy + dy * res) == OCCUPIED:
                    return False
        return True

    def _blocked(self, sign):
        """True if an obstacle spans the robot's actual path ahead of travel.

        sign = +1 for forward motion, -1 for reverse.  Samples the on-axis cell
        and the two box edges at each forward distance, then blocks only when
        the obstacle genuinely blocks the body:

          * the centre cell is occupied (a wall across the heading), or
          * BOTH edges are occupied (a gap the body cannot thread).

        A single corner cell is the door-frame clip: the door is 0.8 m wide and
        the A1 body is 0.4 m, so with FAST-LIO2's ~0.15 m drift a corner sample
        lands on the frame even though the body would still pass.  Blocking on
        any one corner therefore froze the robot in the doorway.  Requiring the
        centre (or both edges) lets a slightly off-centre robot thread the door
        while still catching a wall it is driving straight into."""
        x, y, _, yaw = self.pose
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        res = self.map.info.resolution if self.map is not None else 0.2
        d = self.min_check_dist
        while d <= self.safety_dist + 1e-6:
            occupied = []
            for lat in (-self.safety_half_width, 0.0, self.safety_half_width):
                px = x + sign * d * cos_yaw + lat * -sin_yaw
                py = y + sign * d * sin_yaw + lat * cos_yaw
                occupied.append(self._occ(px, py) == OCCUPIED)
            centre = occupied[1]
            both_edges = occupied[0] and occupied[2]
            if centre and not both_edges:
                # Ignore a floating/phantom obstacle: a real wall occludes the
                # space behind it (UNKNOWN), whereas a phantom cell or cluster
                # left by 3-D flattening has FREE space raytraced beyond it.
                # IMPORTANT: only release when the obstacle is ALSO isolated.
                # A genuine wall (e.g. a door frame while entering a room) has
                # an already-scanned FREE room behind it, so free_behind alone
                # would wrongly treat it as a phantom and the robot would
                # drive straight into the wall.
                if (self._free_behind(x, y, sign, cos_yaw, sin_yaw, d, res) and
                        self._obstacle_is_isolated(x, y, cos_yaw, sin_yaw, d)):
                    centre = False
            if centre or both_edges:
                return True
            d += 0.2
        return False

    def _blocked_forward(self, cmd):
        if self.pose is None or self.in_stairs or cmd.linear.x <= 0.01:
            return False
        # Primary: live laser sees the real obstacle regardless of map state
        # (map walls can be missing under FAST-LIO2 drift -> previously drove
        # into real walls).  Ghost obstacles are not seen by the laser, so they
        # never false-trigger here.
        if self._laser_blocked():
            return True
        # Fallback: occupancy-map check (catches obstacles the laser missed,
        # e.g. angled/occluded views), keeping the phantom-release logic.
        if self.map is not None:
            return self._blocked(+1)
        return False

    def _start_recover(self, now):
        # ALWAYS start by reversing away from the obstacle (the user-visible
        # behaviour: after a collision-safety trip the robot must back off, not
        # freeze).  The reverse phase itself monitors _blocked(-1) every frame
        # and switches to the turn escape the moment backing up would hit
        # something behind, so starting in reverse is always safe.
        self.recover_phase = "reverse"
        self.recover_start_pose = (self.pose[0], self.pose[1])
        self.recover_start_time = now
        self.blocked_since = None
        rospy.logwarn(
            "collision_safety: recovery start (reverse), backing off %.2f m",
            self.reverse_dist,
        )

    def _recover_step(self, now):
        out = Twist()
        if self.recover_phase == "reverse":
            traveled = 0.0
            if self.recover_start_pose is not None and self.pose is not None:
                traveled = math.hypot(
                    self.pose[0] - self.recover_start_pose[0],
                    self.pose[1] - self.recover_start_pose[1],
                )
            elapsed = (now - self.recover_start_time).to_sec()
            if (traveled >= self.reverse_dist or
                    elapsed >= self.reverse_max_time or self._blocked(-1)):
                # Reverse done or reverse also blocked: turn in place to escape
                # a wedge instead of holding forever.
                if self.turn_recovery:
                    self.recover_phase = "turn"
                    self.turn_sign = 1.0
                    self.recover_start_time = now
                    rospy.logwarn(
                        "collision_safety: reverse blocked/done (%.2f m), turning",
                        traveled,
                    )
                else:
                    self.recover_phase = "hold"
                    self.recover_start_time = now
                    rospy.logwarn(
                        "collision_safety: reverse done (%.2f m), holding", traveled
                    )
            else:
                out.linear.x = self.reverse_speed
                out.linear.y = 0.0
                out.angular.z = 0.0
                return out
        if self.recover_phase == "turn":
            if (now - self.recover_start_time).to_sec() >= self.turn_duration:
                self.recover_phase = "hold"
                self.recover_start_time = now
                rospy.logwarn("collision_safety: turn escape done, holding")
            else:
                out.linear.x = 0.0
                out.linear.y = 0.0
                out.angular.z = self.turn_sign * self.turn_yaw_rate
                return out
        if self.recover_phase == "hold":
            if (now - self.recover_start_time).to_sec() >= self.recover_hold_time:
                self.recover_phase = None
                self.last_recover_end = rospy.Time.now()
                rospy.logwarn("collision_safety: recovery finished")
            else:
                out.linear.x = 0.0
                out.linear.y = 0.0
                out.angular.z = 0.0
                return out
        return out

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            cmd = self.last_cmd
            now = rospy.Time.now()
            if self.last_cmd_stamp is not None and now - self.last_cmd_stamp > self.stale_timeout:
                cmd = Twist()  # nav died or stalled -> hold still
                self.recover_phase = None
                self.blocked_since = None
                rospy.logwarn_throttle(2.0, "collision_safety: cmd_vel stale, holding zero")
            elif self.recover_phase is not None:
                # Active recovery manoeuvre overrides the navigation command.
                cmd = self._recover_step(now)
            elif self._blocked_forward(cmd):
                # Forward motion blocked by a real (non-phantom) obstacle.
                if not self.recover_enable:
                    cmd.linear.x = 0.0
                    cmd.linear.y = 0.0
                    rospy.logwarn_throttle(
                        1.0, "collision_safety: forward blocked, zeroing velocity"
                    )
                else:
                    if self.blocked_since is None:
                        self.blocked_since = now
                    if (now - self.blocked_since) < rospy.Duration(self.recover_confirm_time):
                        # Still confirming a persistent block; stop immediately
                        # but keep yaw so the robot can still turn away.
                        cmd.linear.x = 0.0
                        cmd.linear.y = 0.0
                    elif (now - self.last_recover_end) >= rospy.Duration(self.recover_cooldown):
                        self._start_recover(now)
                        cmd = self._recover_step(now)
                    else:
                        # Cooldown: hold still so the navigation node's stall
                        # detection fires and re-plans around the obstacle.
                        cmd.linear.x = 0.0
                        cmd.linear.y = 0.0
                        rospy.logwarn_throttle(
                            2.0, "collision_safety: forward blocked (recovery cooldown), holding"
                        )
            elif (cmd.linear.x < -0.01 and self.pose is not None and
                    self.map is not None and not self.in_stairs and
                    self._blocked(-1)):
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                rospy.logwarn_throttle(
                    1.0, "collision_safety: reverse blocked, zeroing velocity"
                )
            else:
                self.blocked_since = None
            self.pub.publish(cmd)
            r.sleep()


def main():
    rospy.init_node("collision_safety")
    CollisionSafety().spin()


if __name__ == "__main__":
    main()
