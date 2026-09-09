cmake_minimum_required(VERSION 3.0.2)
project(hazard_perception)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_geometry
  message_filters
  message_generation
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  HazardDetection2D.msg
  HazardDetection2DArray.msg
  Hazard3D.msg
  Hazard3DArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    geometry_msgs
    image_geometry
    message_filters
    message_runtime
    nav_msgs
    rospy
    sensor_msgs
    std_msgs
    std_srvs
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/hazard_detector_node.py
    scripts/hazard_odom_tf_bridge.py
    scripts/hazard_pipeline_node.py
    scripts/low_obstacle_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
