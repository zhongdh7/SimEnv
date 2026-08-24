cmake_minimum_required(VERSION 3.0.2)
project(carto_localization)

# Only standard ROS deps are listed here so the SimEnv workspace builds this
# package even though cartographer_ros lives in a separate isolated install.
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/pointcloud_to_laserscan.py
  scripts/odom_dedup.py
  scripts/carto_odom_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
