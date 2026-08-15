cmake_minimum_required(VERSION 3.0.2)
project(fastlio_slam_merge)

find_package(catkin REQUIRED)

catkin_package()

## Install the Python nodes into the package's binary destination so they can
## be launched with roslaunch / rosrun.
catkin_install_python(PROGRAMS
  scripts/pointcloud_converter.py
  scripts/frame_republisher.py
  scripts/save_pcd.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch and config into the package share destination.
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
