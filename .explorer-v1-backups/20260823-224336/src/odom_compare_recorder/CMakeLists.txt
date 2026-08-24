cmake_minimum_required(VERSION 3.0.2)
project(odom_compare_recorder)

find_package(catkin REQUIRED)

catkin_package()

## Install the recorder node so it can be launched with roslaunch / rosrun.
catkin_install_python(PROGRAMS
  scripts/recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files into the package share destination.
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
