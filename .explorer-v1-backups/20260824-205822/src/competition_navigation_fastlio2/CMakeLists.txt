cmake_minimum_required(VERSION 3.14)
project(competition_navigation_fastlio2)

# Script/launch-only package.  The HDPlanner inference library
# (libhdplanner_inference.so) is NOT rebuilt here; the launch reuses the copy
# produced by the competition_navigation package to avoid a CUDA/libtorch
# rebuild and a shared-library name collision.

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/competition_navigation_node.py
  scripts/scan_tf_relay.py
  scripts/collision_safety.py
  tools/full_map_verifier.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

configure_file(
  scripts/corridor_graph.py
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/corridor_graph.py
  COPYONLY
)

install(FILES scripts/corridor_graph.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS tools/export_hdplanner_torchscript.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tools
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_full_map_logic.py)
  catkin_add_nosetests(test/test_corridor_graph.py)
endif()
