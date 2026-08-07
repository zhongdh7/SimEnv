cmake_minimum_required(VERSION 3.0.2)
project(dwa_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# Install RViz config
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

# Install Python scripts (devel-space symlink + install manifest)
catkin_install_python(PROGRAMS
  scripts/pointcloud_to_laserscan.py
  scripts/auto_room_explorer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install maps
install(DIRECTORY maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
)
