cmake_minimum_required(VERSION 3.0.2)
project(odom_compare_player)

find_package(catkin REQUIRED)

catkin_package()

## Install the Python nodes and the shared helper module.  They land in the
## same lib/<pkg>/ directory, so the scripts import odom_compare_lib via
## sys.path relative to __file__.
catkin_install_python(PROGRAMS
  scripts/odom_compare_lib.py
  scripts/visualizer.py
  scripts/animate.py
  scripts/analyze.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch and RViz config into the package share destination.
install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
