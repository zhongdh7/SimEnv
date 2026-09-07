cmake_minimum_required(VERSION 3.0.2)
project(competition_navigation_hazard_v0)

# Launch-only integration package: it does not build any node.  It exists to
# provide a single roslaunch entry point that starts the C++ navigation stack
# and the hazard-perception-v0 pipeline together.  All behaviour lives in the
# two packages it depends on.
#
# This is the v0-hazard variant of competition_navigation_hazard: the only
# difference is the hazard detection is provided by hazard_perception_v0
# (copied from explorer-v0) instead of hazard_perception.  Navigation and
# completion logic are identical.

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS scripts/hazard_finalize_trigger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
