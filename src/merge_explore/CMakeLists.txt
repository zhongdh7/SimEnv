cmake_minimum_required(VERSION 3.0.2)
project(merge_explore)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Python scripts
catkin_install_python(PROGRAMS
  scripts/stair_climber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
