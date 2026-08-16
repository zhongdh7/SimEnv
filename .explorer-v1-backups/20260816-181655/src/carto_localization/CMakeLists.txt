cmake_minimum_required(VERSION 3.0.2)
project(carto_localization)

find_package(catkin REQUIRED)

catkin_package()

## Install the Python node into the package's binary destination so it can be
## launched with roslaunch / rosrun.
catkin_install_python(PROGRAMS
  scripts/pointcloud_to_laserscan.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch, config and rviz into the package share destination.
install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
