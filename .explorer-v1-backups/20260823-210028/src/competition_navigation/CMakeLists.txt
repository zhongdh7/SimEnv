cmake_minimum_required(VERSION 3.14)
project(competition_navigation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc CACHE FILEPATH "CUDA compiler")
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES 120 CACHE STRING "CUDA architectures")
endif()
list(APPEND CMAKE_PREFIX_PATH "/opt/libtorch/current")
set(Torch_DIR "/opt/libtorch/current/share/cmake/Torch" CACHE PATH "Torch CMake package")
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
)

catkin_package(
  LIBRARIES hdplanner_inference
)

add_library(hdplanner_inference SHARED
  src/hdplanner_inference.cpp
)
target_link_libraries(hdplanner_inference
  ${TORCH_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/competition_navigation_node.py
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

install(TARGETS hdplanner_inference
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
