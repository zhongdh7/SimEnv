#!/usr/bin/env python3
"""Listen for exploration completion and auto-finalize the hazard results.

The C++ navigation stack and hazard_perception run independently (shallow
integration).  This node watches /competition_navigation/status; once the full
3-floor map is complete (status contains "full_map_complete" or "returned_home")
it calls the /hazard_perception/finalize service so the detected hazard
coordinates are written to the results directory automatically — no manual
`rosservice call` needed.

It triggers at most once per run (the pipeline rejects a second finalize).
"""

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger

_FINALIZE_SERVICE = "/hazard_perception/finalize"
# Completion markers in the navigation status string.
_DONE_MARKERS = ("full_map_complete", "returned_home")


class FinalizeTrigger:
    def __init__(self):
        self._triggered = False
        self._service = rospy.ServiceProxy(_FINALIZE_SERVICE, Trigger)
        self._sub = rospy.Subscriber(
            "/competition_navigation/status", String, self._on_status
        )
        rospy.loginfo("hazard_finalize_trigger: watching nav status for completion")

    def _on_status(self, msg):
        if self._triggered:
            return
        status = msg.data or ""
        if not any(marker in status for marker in _DONE_MARKERS):
            return
        self._triggered = True
        try:
            # Give the perception node a moment to flush before finalize.
            if not rospy.is_shutdown():
                rospy.sleep(1.0)
            resp = self._service()
            rospy.loginfo(
                "hazard_finalize_trigger: finalized (success=%s, msg=%s)",
                resp.success, resp.message,
            )
        except rospy.ServiceException as exc:
            rospy.logwarn("hazard_finalize_trigger: finalize call failed: %s", exc)
        except rospy.ROSInterruptException:
            pass


def main():
    rospy.init_node("hazard_finalize_trigger")
    FinalizeTrigger()
    rospy.spin()


if __name__ == "__main__":
    main()
