cmake_minimum_required(VERSION 2.8.3)
project(livox_laser_simulation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})

#To solve the error which gazebo does not handle the include file well, we must add this line.
# Gazebo include path, auto-detected from GAZEBO_INCLUDE_DIRS or fallback to default
if(DEFINED ENV{GAZEBO_VERSION})
    set(GAZEBO_VER "$ENV{GAZEBO_VERSION}")
else()
    set(GAZEBO_VER "11")
endif()
include_directories(/usr/include/gazebo-${GAZEBO_VER}/gazebo)

link_directories(${GAZEBO_LIBRARY_DIRS})

#This is designed for whose proto installed in system is not 2.6. We can install the version of proto in local dir
#include_directories($ENV{PROTO_ROOT}/include/)  # Set PROTO_ROOT env var if using custom proto
#link_directories($ENV{PROTO_ROOT}/lib/)  # Set PROTO_ROOT env var if using custom proto

include_directories(
         include
        ${catkin_INCLUDE_DIRS}
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES livox_laser_simulation
        CATKIN_DEPENDS tf
)

add_library(livox_laser_simulation SHARED src/livox_points_plugin.cpp src/livox_ode_multiray_shape.cpp)
target_link_libraries(livox_laser_simulation ${catkin_LIBRARIES} RayPlugin)

target_link_libraries(livox_laser_simulation libprotobuf.so.9)
