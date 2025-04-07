# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "drivers: 0 messages, 2 services")

set(MSG_I_FLAGS "-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Itrajectory_msgs:/opt/ros/noetic/share/trajectory_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(drivers_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/robot/Documents/2024-2-Projeto2/obelix/robot_ws/src/drivers/srv/Camera.srv" NAME_WE)
add_custom_target(_drivers_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "drivers" "/home/robot/Documents/2024-2-Projeto2/obelix/robot_ws/src/drivers/srv/Camera.srv" "std_msgs/Header:sensor_msgs/Image"
)

get_filename_component(_filename "/home/robot/Documents/2024-2-Projeto2/obelix/robot_ws/src/drivers/srv/Servo.srv" NAME_WE)
add_custom_target(_drivers_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "drivers" "/home/robot/Documents/2024-2-Projeto2/obelix/robot_ws/src/drivers/srv/Servo.srv" "trajectory_msgs/JointTrajectoryPoint:std_msgs/Header:trajectory_msgs/JointTrajectory"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(drivers
  "/home/robot/Documents/2024-2-Projeto2/obelix/robot_ws/src/drivers/srv/Camera.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drivers
)
_generate_srv_cpp(drivers
  "/home/robot/Documents/2024-2-Projeto2/obelix/robot_ws/src/drivers/srv/Servo.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drivers
)

### Generating Module File
_generate_module_cpp(drivers
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drivers
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(drivers_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(drivers_generate_messages drivers_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robot/Documents/2024-2-Projeto2/obelix/robot_ws/src/drivers/srv/Camera.srv" NAME_WE)
add_dependencies(drivers_generate_messages_cpp _drivers_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot/Documents/2024-2-Projeto2/obelix/robot_ws/src/drivers/srv/Servo.srv" NAME_WE)
add_dependencies(drivers_generate_messages_cpp _drivers_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(drivers_gencpp)
add_dependencies(drivers_gencpp drivers_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS drivers_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(drivers
  "/home/robot/Documents/2024-2-Projeto2/obelix/robot_ws/src/drivers/srv/Camera.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drivers
)
_generate_srv_eus(drivers
  "/home/robot/Documents/2024-2-Projeto2/obelix/robot_ws/src/drivers/srv/Servo.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drivers
)

### Generating Module File
_generate_module_eus(drivers
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drivers
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(drivers_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(drivers_generate_messages drivers_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robot/Documents/2024-2-Projeto2/obelix/robot_ws/src/drivers/srv/Camera.srv" NAME_WE)
add_dependencies(drivers_generate_messages_eus _drivers_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot/Documents/2024-2-Projeto2/obelix/robot_ws/src/drivers/srv/Servo.srv" NAME_WE)
add_dependencies(drivers_generate_messages_eus _drivers_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(drivers_geneus)
add_dependencies(drivers_geneus drivers_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS drivers_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(drivers
  "/home/robot/Documents/2024-2-Projeto2/obelix/robot_ws/src/drivers/srv/Camera.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drivers
)
_generate_srv_lisp(drivers
  "/home/robot/Documents/2024-2-Projeto2/obelix/robot_ws/src/drivers/srv/Servo.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drivers
)

### Generating Module File
_generate_module_lisp(drivers
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drivers
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(drivers_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(drivers_generate_messages drivers_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robot/Documents/2024-2-Projeto2/obelix/robot_ws/src/drivers/srv/Camera.srv" NAME_WE)
add_dependencies(drivers_generate_messages_lisp _drivers_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot/Documents/2024-2-Projeto2/obelix/robot_ws/src/drivers/srv/Servo.srv" NAME_WE)
add_dependencies(drivers_generate_messages_lisp _drivers_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(drivers_genlisp)
add_dependencies(drivers_genlisp drivers_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS drivers_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(drivers
  "/home/robot/Documents/2024-2-Projeto2/obelix/robot_ws/src/drivers/srv/Camera.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drivers
)
_generate_srv_nodejs(drivers
  "/home/robot/Documents/2024-2-Projeto2/obelix/robot_ws/src/drivers/srv/Servo.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drivers
)

### Generating Module File
_generate_module_nodejs(drivers
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drivers
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(drivers_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(drivers_generate_messages drivers_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robot/Documents/2024-2-Projeto2/obelix/robot_ws/src/drivers/srv/Camera.srv" NAME_WE)
add_dependencies(drivers_generate_messages_nodejs _drivers_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot/Documents/2024-2-Projeto2/obelix/robot_ws/src/drivers/srv/Servo.srv" NAME_WE)
add_dependencies(drivers_generate_messages_nodejs _drivers_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(drivers_gennodejs)
add_dependencies(drivers_gennodejs drivers_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS drivers_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(drivers
  "/home/robot/Documents/2024-2-Projeto2/obelix/robot_ws/src/drivers/srv/Camera.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drivers
)
_generate_srv_py(drivers
  "/home/robot/Documents/2024-2-Projeto2/obelix/robot_ws/src/drivers/srv/Servo.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drivers
)

### Generating Module File
_generate_module_py(drivers
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drivers
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(drivers_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(drivers_generate_messages drivers_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robot/Documents/2024-2-Projeto2/obelix/robot_ws/src/drivers/srv/Camera.srv" NAME_WE)
add_dependencies(drivers_generate_messages_py _drivers_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot/Documents/2024-2-Projeto2/obelix/robot_ws/src/drivers/srv/Servo.srv" NAME_WE)
add_dependencies(drivers_generate_messages_py _drivers_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(drivers_genpy)
add_dependencies(drivers_genpy drivers_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS drivers_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drivers)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drivers
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(drivers_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(drivers_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET trajectory_msgs_generate_messages_cpp)
  add_dependencies(drivers_generate_messages_cpp trajectory_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drivers)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drivers
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(drivers_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(drivers_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET trajectory_msgs_generate_messages_eus)
  add_dependencies(drivers_generate_messages_eus trajectory_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drivers)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drivers
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(drivers_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(drivers_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET trajectory_msgs_generate_messages_lisp)
  add_dependencies(drivers_generate_messages_lisp trajectory_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drivers)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drivers
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(drivers_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(drivers_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET trajectory_msgs_generate_messages_nodejs)
  add_dependencies(drivers_generate_messages_nodejs trajectory_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drivers)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drivers\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drivers
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(drivers_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(drivers_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET trajectory_msgs_generate_messages_py)
  add_dependencies(drivers_generate_messages_py trajectory_msgs_generate_messages_py)
endif()
