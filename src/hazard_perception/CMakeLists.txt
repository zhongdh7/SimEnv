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
    tools/evaluate_floor0.py
    tools/evaluate_floor0_visibility.py
    tools/evaluate_floor0_witness.py
    tools/replay_floor0_tracker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_detector.py)
  catkin_add_nosetests(test/test_localization.py)
  catkin_add_nosetests(test/test_tracker.py)
  catkin_add_nosetests(test/test_livox_fusion.py)
  catkin_add_nosetests(test/test_result_manager.py)
  catkin_add_nosetests(test/test_evaluate_floor0.py)
  catkin_add_nosetests(test/test_visibility_audit.py)
endif()
