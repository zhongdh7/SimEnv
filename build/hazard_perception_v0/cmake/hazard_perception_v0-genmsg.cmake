# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "hazard_perception_v0: 4 messages, 0 services")

set(MSG_I_FLAGS "-Ihazard_perception_v0:/home/loser/SimEnv/src/hazard_perception_v0/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(hazard_perception_v0_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2D.msg" NAME_WE)
add_custom_target(_hazard_perception_v0_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hazard_perception_v0" "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2D.msg" ""
)

get_filename_component(_filename "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2DArray.msg" NAME_WE)
add_custom_target(_hazard_perception_v0_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hazard_perception_v0" "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2DArray.msg" "hazard_perception_v0/HazardDetection2D:std_msgs/Header"
)

get_filename_component(_filename "/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3D.msg" NAME_WE)
add_custom_target(_hazard_perception_v0_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hazard_perception_v0" "/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3D.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3DArray.msg" NAME_WE)
add_custom_target(_hazard_perception_v0_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hazard_perception_v0" "/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3DArray.msg" "std_msgs/Header:hazard_perception_v0/Hazard3D:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(hazard_perception_v0
  "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hazard_perception_v0
)
_generate_msg_cpp(hazard_perception_v0
  "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2DArray.msg"
  "${MSG_I_FLAGS}"
  "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2D.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hazard_perception_v0
)
_generate_msg_cpp(hazard_perception_v0
  "/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hazard_perception_v0
)
_generate_msg_cpp(hazard_perception_v0
  "/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3DArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hazard_perception_v0
)

### Generating Services

### Generating Module File
_generate_module_cpp(hazard_perception_v0
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hazard_perception_v0
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(hazard_perception_v0_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(hazard_perception_v0_generate_messages hazard_perception_v0_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2D.msg" NAME_WE)
add_dependencies(hazard_perception_v0_generate_messages_cpp _hazard_perception_v0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2DArray.msg" NAME_WE)
add_dependencies(hazard_perception_v0_generate_messages_cpp _hazard_perception_v0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3D.msg" NAME_WE)
add_dependencies(hazard_perception_v0_generate_messages_cpp _hazard_perception_v0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3DArray.msg" NAME_WE)
add_dependencies(hazard_perception_v0_generate_messages_cpp _hazard_perception_v0_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hazard_perception_v0_gencpp)
add_dependencies(hazard_perception_v0_gencpp hazard_perception_v0_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hazard_perception_v0_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(hazard_perception_v0
  "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hazard_perception_v0
)
_generate_msg_eus(hazard_perception_v0
  "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2DArray.msg"
  "${MSG_I_FLAGS}"
  "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2D.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hazard_perception_v0
)
_generate_msg_eus(hazard_perception_v0
  "/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hazard_perception_v0
)
_generate_msg_eus(hazard_perception_v0
  "/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3DArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hazard_perception_v0
)

### Generating Services

### Generating Module File
_generate_module_eus(hazard_perception_v0
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hazard_perception_v0
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(hazard_perception_v0_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(hazard_perception_v0_generate_messages hazard_perception_v0_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2D.msg" NAME_WE)
add_dependencies(hazard_perception_v0_generate_messages_eus _hazard_perception_v0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2DArray.msg" NAME_WE)
add_dependencies(hazard_perception_v0_generate_messages_eus _hazard_perception_v0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3D.msg" NAME_WE)
add_dependencies(hazard_perception_v0_generate_messages_eus _hazard_perception_v0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3DArray.msg" NAME_WE)
add_dependencies(hazard_perception_v0_generate_messages_eus _hazard_perception_v0_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hazard_perception_v0_geneus)
add_dependencies(hazard_perception_v0_geneus hazard_perception_v0_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hazard_perception_v0_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(hazard_perception_v0
  "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hazard_perception_v0
)
_generate_msg_lisp(hazard_perception_v0
  "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2DArray.msg"
  "${MSG_I_FLAGS}"
  "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2D.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hazard_perception_v0
)
_generate_msg_lisp(hazard_perception_v0
  "/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hazard_perception_v0
)
_generate_msg_lisp(hazard_perception_v0
  "/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3DArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hazard_perception_v0
)

### Generating Services

### Generating Module File
_generate_module_lisp(hazard_perception_v0
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hazard_perception_v0
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(hazard_perception_v0_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(hazard_perception_v0_generate_messages hazard_perception_v0_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2D.msg" NAME_WE)
add_dependencies(hazard_perception_v0_generate_messages_lisp _hazard_perception_v0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2DArray.msg" NAME_WE)
add_dependencies(hazard_perception_v0_generate_messages_lisp _hazard_perception_v0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3D.msg" NAME_WE)
add_dependencies(hazard_perception_v0_generate_messages_lisp _hazard_perception_v0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3DArray.msg" NAME_WE)
add_dependencies(hazard_perception_v0_generate_messages_lisp _hazard_perception_v0_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hazard_perception_v0_genlisp)
add_dependencies(hazard_perception_v0_genlisp hazard_perception_v0_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hazard_perception_v0_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(hazard_perception_v0
  "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hazard_perception_v0
)
_generate_msg_nodejs(hazard_perception_v0
  "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2DArray.msg"
  "${MSG_I_FLAGS}"
  "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2D.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hazard_perception_v0
)
_generate_msg_nodejs(hazard_perception_v0
  "/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hazard_perception_v0
)
_generate_msg_nodejs(hazard_perception_v0
  "/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3DArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hazard_perception_v0
)

### Generating Services

### Generating Module File
_generate_module_nodejs(hazard_perception_v0
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hazard_perception_v0
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(hazard_perception_v0_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(hazard_perception_v0_generate_messages hazard_perception_v0_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2D.msg" NAME_WE)
add_dependencies(hazard_perception_v0_generate_messages_nodejs _hazard_perception_v0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2DArray.msg" NAME_WE)
add_dependencies(hazard_perception_v0_generate_messages_nodejs _hazard_perception_v0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3D.msg" NAME_WE)
add_dependencies(hazard_perception_v0_generate_messages_nodejs _hazard_perception_v0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3DArray.msg" NAME_WE)
add_dependencies(hazard_perception_v0_generate_messages_nodejs _hazard_perception_v0_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hazard_perception_v0_gennodejs)
add_dependencies(hazard_perception_v0_gennodejs hazard_perception_v0_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hazard_perception_v0_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(hazard_perception_v0
  "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hazard_perception_v0
)
_generate_msg_py(hazard_perception_v0
  "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2DArray.msg"
  "${MSG_I_FLAGS}"
  "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2D.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hazard_perception_v0
)
_generate_msg_py(hazard_perception_v0
  "/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hazard_perception_v0
)
_generate_msg_py(hazard_perception_v0
  "/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3DArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hazard_perception_v0
)

### Generating Services

### Generating Module File
_generate_module_py(hazard_perception_v0
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hazard_perception_v0
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(hazard_perception_v0_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(hazard_perception_v0_generate_messages hazard_perception_v0_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2D.msg" NAME_WE)
add_dependencies(hazard_perception_v0_generate_messages_py _hazard_perception_v0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/loser/SimEnv/src/hazard_perception_v0/msg/HazardDetection2DArray.msg" NAME_WE)
add_dependencies(hazard_perception_v0_generate_messages_py _hazard_perception_v0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3D.msg" NAME_WE)
add_dependencies(hazard_perception_v0_generate_messages_py _hazard_perception_v0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/loser/SimEnv/src/hazard_perception_v0/msg/Hazard3DArray.msg" NAME_WE)
add_dependencies(hazard_perception_v0_generate_messages_py _hazard_perception_v0_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hazard_perception_v0_genpy)
add_dependencies(hazard_perception_v0_genpy hazard_perception_v0_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hazard_perception_v0_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hazard_perception_v0)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hazard_perception_v0
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(hazard_perception_v0_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(hazard_perception_v0_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hazard_perception_v0)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hazard_perception_v0
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(hazard_perception_v0_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(hazard_perception_v0_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hazard_perception_v0)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hazard_perception_v0
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(hazard_perception_v0_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(hazard_perception_v0_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hazard_perception_v0)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hazard_perception_v0
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(hazard_perception_v0_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(hazard_perception_v0_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hazard_perception_v0)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hazard_perception_v0\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hazard_perception_v0
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  string(REGEX REPLACE "([][+.*()^])" "\\\\\\1" ESCAPED_PATH "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hazard_perception_v0")
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hazard_perception_v0
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${ESCAPED_PATH}/.+/__init__.pyc?$"
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(hazard_perception_v0_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(hazard_perception_v0_generate_messages_py std_msgs_generate_messages_py)
endif()
