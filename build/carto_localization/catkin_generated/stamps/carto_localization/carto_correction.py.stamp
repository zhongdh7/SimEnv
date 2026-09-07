#!/usr/bin/env python3
"""Compute a smooth x/y drift-correction for FAST-LIO2 from Cartographer's
loop-closure-corrected pose, without a feedback loop.

Why this exists
---------------
FAST-LIO2 (no loop closure) drifts x/y over a run; Cartographer (2-D, odometry-
aided) loop-closure-corrects long-term drift but its 2-D scan matcher is
rotation-degenerate in the featureless corridor, so it must NOT become the
navigation's live pose.  The navigation therefore keeps FAST-LIO2 as its realtime
pose source (scan_tf_relay -> /Odometry_fastlio) and this node folds in ONLY the
x/y translation that Cartographer believes FAST-LIO2 has drifted.

No feedback loop
----------------
This node reads the RAW FAST-LIO2 odometry (/Odometry, camera_init frame) and
anchors it to the building-aligned odom frame itself (one-time GT anchor,
exactly like scan_tf_relay), then compares it with Cartographer's corrected
pose (/carto_odom from carto_relay, already in odom).  Both are ``base in odom``
and share the same starting anchor, so the difference is purely the drift that
Cartographer's loop closure removed.  scan_tf_relay only *receives* the
resulting delta; it never feeds back into this node.

Output
------
geometry_msgs/Point on ~delta_topic (default /carto_correction/delta):
  x = delta_x (m), y = delta_y (m).  Smoothed with an exponential low-pass and
  rate-limited so the correction cannot jump the robot's map.

Only x/y is corrected (per decision); yaw stays FAST-LIO2's.
"""

from __future__ import print_function

import math
import threading

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

GT_TOPIC_DEFAULT = "/Odometry_gazebo"


def _finite(vals):
    return all(math.isfinite(float(v)) for v in vals)


class CartoCorrection(object):
    def __init__(self):
        self.fastlio_topic = rospy.get_param("~fastlio_odom_topic", "/Odometry")
        self.carto_topic = rospy.get_param("~carto_odom_topic", "/carto_odom")
        self.gt_topic = rospy.get_param("~gt_odom_topic", GT_TOPIC_DEFAULT)
        self.delta_topic = rospy.get_param("~delta_topic",
                                           "/carto_correction/delta")
        self.rate_hz = float(rospy.get_param("~rate", 2.0))
        # Exponential smoothing: alpha=1 -> instant, ~0 -> very slow.
        self.alpha = float(rospy.get_param("~alpha", 0.08))
        # Max per-second correction (m) to avoid jumps.
        self.max_step = float(rospy.get_param("~max_step_per_s", 0.02))
        # Hard cap on the total correction magnitude (m).  Measured FAST-LIO2
        # drift is ~0.5 m over a full-map run, and Cartographer 2-D itself is
        # only good to ~0.7 m in this corridor, so a delta beyond ~0.3 m is
        # more likely Cartographer's own error than real FAST-LIO2 drift.
        # Clamp keeps an unreliable Cartographer pose from yanking the nav.
        self.max_delta = float(rospy.get_param("~max_delta", 0.30))
        # Ignore carto pose until its transform has been stable this long.
        self.settle_delay = float(rospy.get_param("~settle_delay", 10.0))

        self._lock = threading.Lock()
        self._fastlio = None      # (t, q) base in odom, from raw FAST-LIO2
        self._carto = None        # (t, q) base in odom, from /carto_odom
        self._gt = None           # latest (t, q) base in odom from GT

        # One-time anchor for raw FAST-LIO2 camera_init -> odom.
        self._aligned = False
        self._q_oc = None
        self._t_oc = None

        self._delta = (0.0, 0.0)   # current smoothed (dx, dy)
        self._last_cb_time = None
        self._start_time = rospy.Time.now().to_sec()

        rospy.Subscriber(self.fastlio_topic, Odometry, self._fastlio_cb,
                         queue_size=5)
        rospy.Subscriber(self.carto_topic, Odometry, self._carto_cb, queue_size=5)
        rospy.Subscriber(self.gt_topic, Odometry, self._gt_cb, queue_size=5)

        self._pub = rospy.Publisher(self.delta_topic, Point, queue_size=1)
        self._timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz),
                                  self._timer_cb)
        rospy.loginfo(
            "carto_correction: fastlio=%s carto=%s -> %s (alpha=%.2f "
            "max_step=%.3f m/s)", self.fastlio_topic, self.carto_topic,
            self.delta_topic, self.alpha, self.max_step)

    # ---- subscribers -----------------------------------------------------

    def _gt_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        if not _finite((p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            return
        with self._lock:
            self._gt = ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def _fastlio_cb(self, msg):
        """Raw FAST-LIO2 odometry: pose of base in frame camera_init."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        if not _finite((p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            return
        with self._lock:
            self._try_anchor(p, q)
            if not self._aligned:
                return
            # Transform camera_init -> odom using the locked anchor.
            t_cb = (p.x, p.y, p.z)
            q_cb = (q.x, q.y, q.z, q.w)
            t_ob, q_ob = self._apply_anchor(t_cb, q_cb)
            self._fastlio = (t_ob, q_ob)

    def _carto_cb(self, msg):
        """carto_relay output: pose of base in odom, loop-closure-corrected."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        if not _finite((p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            return
        with self._lock:
            self._carto = ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    # ---- anchoring (mirror of scan_tf_relay) ------------------------------

    def _try_anchor(self, p_cb, q_cb):
        if self._aligned or self._gt is None:
            return
        t_ob0, q_ob0 = self._gt   # q in (x,y,z,w)
        # Raw FAST-LIO2 odometry child frame is body == base.  We want the
        # odom <- camera_init anchor.  carto_relay already anchors carto_map
        # the same way, so the two base-in-odom poses share the start origin.
        #
        #   q_oc = q_ob(0) * q_cb(0)^-1
        #   t_oc = t_ob(0) - R(q_oc) * t_cb(0)
        q_cb0 = (q_cb.x, q_cb.y, q_cb.z, q_cb.w)
        q_oc = quat_multiply(q_ob0, quat_inverse(q_cb0))
        R_oc = rot_matrix(q_oc)
        t_cb0 = (p_cb.x, p_cb.y, p_cb.z)
        # t_oc = t_ob(0) - R_oc * t_cb(0)
        t_oc = tuple(t_ob0[r] - sum(R_oc[r][c] * t_cb0[c] for c in range(3))
                     for r in range(3))
        self._q_oc = q_oc
        self._t_oc = t_oc
        self._aligned = True
        yaw = math.atan2(2.0 * (q_oc[3] * q_oc[2] + q_oc[0] * q_oc[1]),
                         1.0 - 2.0 * (q_oc[1] * q_oc[1] + q_oc[2] * q_oc[2]))
        rospy.loginfo("carto_correction: FAST-LIO2 anchored to odom "
                      "t_oc=(%.3f,%.3f,%.3f) yaw=%.1f deg",
                      t_oc[0], t_oc[1], t_oc[2], math.degrees(yaw))

    def _apply_anchor(self, t_cb, q_cb):
        q_oc = self._q_oc
        t_oc = self._t_oc
        q_ob = quat_multiply(q_oc, q_cb)
        R_oc = rot_matrix(q_oc)
        t_ob = tuple(t_oc[r] + sum(R_oc[r][c] * t_cb[c] for c in range(3))
                     for r in range(3))
        return t_ob, q_ob

    # ---- correction -------------------------------------------------------

    def _timer_cb(self, _event):
        with self._lock:
            fast = self._fastlio
            carto = self._carto
            aligned = self._aligned
        if not aligned or fast is None or carto is None:
            return
        # Only correct once Cartographer has had time to build a map / settle.
        now = rospy.Time.now().to_sec()
        if now - self._start_time < self.settle_delay:
            return

        # Both are base-in-odom.  Desired correction pulls FAST-LIO2's base onto
        # Cartographer's base in x/y only.
        fx, fy = fast[0][0], fast[0][1]
        cx, cy = carto[0][0], carto[0][1]
        target_dx = cx - fx
        target_dy = cy - fy

        # Rate limit then low-pass smooth.
        if self._last_cb_time is not None:
            dt = now - self._last_cb_time
            if dt > 0.0:
                step = self.max_step * dt
                cur_x, cur_y = self._delta
                dx = clamp(target_dx, cur_x - step, cur_x + step)
                dy = clamp(target_dy, cur_y - step, cur_y + step)
                self._delta = (cur_x + self.alpha * (dx - cur_x),
                               cur_y + self.alpha * (dy - cur_y))
        else:
            self._delta = (0.0, 0.0)
        self._last_cb_time = now

        # Clamp the correction vector magnitude to max_delta.
        dx, dy = self._delta
        mag = math.hypot(dx, dy)
        if mag > self.max_delta:
            scale = self.max_delta / mag
            dx *= scale
            dy *= scale
            self._delta = (dx, dy)

        msg = Point()
        msg.x, msg.y = self._delta
        self._pub.publish(msg)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def quat_multiply(q1, q2):
    """Multiply quaternions given as (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quat_inverse(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def rot_matrix(q):
    x, y, z, w = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def main():
    rospy.init_node("carto_correction")
    CartoCorrection()
    rospy.spin()


if __name__ == "__main__":
    main()
