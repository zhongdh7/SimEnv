cmake_minimum_required(VERSION 3.14)
project(competition_navigation_fastlio2_cpp)

# C++ port of competition_navigation_fastlio2.  The navigation logic is a
# 1:1 translation of the Python package; only the language changed.  The
# HDPlanner inference library (libhdplanner_inference.so) is NOT rebuilt here:
# it is loaded at runtime via dlopen from the ``library_path`` roslaunch param,
# exactly as the Python version did with ctypes.CDLL, and it reuses the copy
# produced by the competition_navigation package (avoids a CUDA/libtorch
# rebuild and a shared-library name collision).

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    std_srvs
    tf2_ros
    tf2_geometry_msgs
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Graph topology helper (the C++ equivalent of corridor_graph.py).
add_library(corridor_graph
  src/corridor_graph.cpp
)
target_link_libraries(corridor_graph ${catkin_LIBRARIES})

# Runtime dlopen wrapper for the HDPlanner C bridge.
add_library(hdplanner_policy
  src/hdplanner_policy.cpp
)
target_link_libraries(hdplanner_policy ${catkin_LIBRARIES} ${CMAKE_DL_LIBS})

# Navigation main node (port of competition_navigation_node.py).
add_executable(competition_navigation_node
  src/competition_navigation_node.cpp
)
target_link_libraries(competition_navigation_node
  corridor_graph
  hdplanner_policy
  ${catkin_LIBRARIES}
  ${CMAKE_DL_LIBS}
)

# FAST-LIO2 -> odom re-anchoring relay (port of scan_tf_relay.py).
add_executable(scan_tf_relay
  src/scan_tf_relay.cpp
)
target_link_libraries(scan_tf_relay ${catkin_LIBRARIES})

# Collision-safety layer (port of collision_safety.py).
add_executable(collision_safety
  src/collision_safety.cpp
)
target_link_libraries(collision_safety ${catkin_LIBRARIES})

install(TARGETS
  corridor_graph
  hdplanner_policy
  competition_navigation_node
  scan_tf_relay
  collision_safety
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
