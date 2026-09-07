cmake_minimum_required(VERSION 3.0.2)
project(competition_navigation_hazard)

# Launch-only integration package: it does not build any node.  It exists to
# provide a single roslaunch entry point that starts the C++ navigation stack
# and the hazard-perception pipeline together.  All behaviour lives in the two
# packages it depends on.

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
