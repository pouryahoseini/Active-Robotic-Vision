# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "active_vision_msgs: 1 messages, 1 services")

set(MSG_I_FLAGS "-Iactive_vision_msgs:/home/pourya/Documents/Active_Perception/src/vision_msgs/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(active_vision_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/pourya/Documents/Active_Perception/src/vision_msgs/msg/Vision_Message.msg" NAME_WE)
add_custom_target(_active_vision_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "active_vision_msgs" "/home/pourya/Documents/Active_Perception/src/vision_msgs/msg/Vision_Message.msg" "geometry_msgs/Point32"
)

get_filename_component(_filename "/home/pourya/Documents/Active_Perception/src/vision_msgs/srv/Vision_Service.srv" NAME_WE)
add_custom_target(_active_vision_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "active_vision_msgs" "/home/pourya/Documents/Active_Perception/src/vision_msgs/srv/Vision_Service.srv" "active_vision_msgs/Vision_Message:geometry_msgs/Point32"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(active_vision_msgs
  "/home/pourya/Documents/Active_Perception/src/vision_msgs/msg/Vision_Message.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/active_vision_msgs
)

### Generating Services
_generate_srv_cpp(active_vision_msgs
  "/home/pourya/Documents/Active_Perception/src/vision_msgs/srv/Vision_Service.srv"
  "${MSG_I_FLAGS}"
  "/home/pourya/Documents/Active_Perception/src/vision_msgs/msg/Vision_Message.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/active_vision_msgs
)

### Generating Module File
_generate_module_cpp(active_vision_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/active_vision_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(active_vision_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(active_vision_msgs_generate_messages active_vision_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pourya/Documents/Active_Perception/src/vision_msgs/msg/Vision_Message.msg" NAME_WE)
add_dependencies(active_vision_msgs_generate_messages_cpp _active_vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pourya/Documents/Active_Perception/src/vision_msgs/srv/Vision_Service.srv" NAME_WE)
add_dependencies(active_vision_msgs_generate_messages_cpp _active_vision_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(active_vision_msgs_gencpp)
add_dependencies(active_vision_msgs_gencpp active_vision_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS active_vision_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(active_vision_msgs
  "/home/pourya/Documents/Active_Perception/src/vision_msgs/msg/Vision_Message.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/active_vision_msgs
)

### Generating Services
_generate_srv_eus(active_vision_msgs
  "/home/pourya/Documents/Active_Perception/src/vision_msgs/srv/Vision_Service.srv"
  "${MSG_I_FLAGS}"
  "/home/pourya/Documents/Active_Perception/src/vision_msgs/msg/Vision_Message.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/active_vision_msgs
)

### Generating Module File
_generate_module_eus(active_vision_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/active_vision_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(active_vision_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(active_vision_msgs_generate_messages active_vision_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pourya/Documents/Active_Perception/src/vision_msgs/msg/Vision_Message.msg" NAME_WE)
add_dependencies(active_vision_msgs_generate_messages_eus _active_vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pourya/Documents/Active_Perception/src/vision_msgs/srv/Vision_Service.srv" NAME_WE)
add_dependencies(active_vision_msgs_generate_messages_eus _active_vision_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(active_vision_msgs_geneus)
add_dependencies(active_vision_msgs_geneus active_vision_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS active_vision_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(active_vision_msgs
  "/home/pourya/Documents/Active_Perception/src/vision_msgs/msg/Vision_Message.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/active_vision_msgs
)

### Generating Services
_generate_srv_lisp(active_vision_msgs
  "/home/pourya/Documents/Active_Perception/src/vision_msgs/srv/Vision_Service.srv"
  "${MSG_I_FLAGS}"
  "/home/pourya/Documents/Active_Perception/src/vision_msgs/msg/Vision_Message.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/active_vision_msgs
)

### Generating Module File
_generate_module_lisp(active_vision_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/active_vision_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(active_vision_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(active_vision_msgs_generate_messages active_vision_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pourya/Documents/Active_Perception/src/vision_msgs/msg/Vision_Message.msg" NAME_WE)
add_dependencies(active_vision_msgs_generate_messages_lisp _active_vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pourya/Documents/Active_Perception/src/vision_msgs/srv/Vision_Service.srv" NAME_WE)
add_dependencies(active_vision_msgs_generate_messages_lisp _active_vision_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(active_vision_msgs_genlisp)
add_dependencies(active_vision_msgs_genlisp active_vision_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS active_vision_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(active_vision_msgs
  "/home/pourya/Documents/Active_Perception/src/vision_msgs/msg/Vision_Message.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/active_vision_msgs
)

### Generating Services
_generate_srv_nodejs(active_vision_msgs
  "/home/pourya/Documents/Active_Perception/src/vision_msgs/srv/Vision_Service.srv"
  "${MSG_I_FLAGS}"
  "/home/pourya/Documents/Active_Perception/src/vision_msgs/msg/Vision_Message.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/active_vision_msgs
)

### Generating Module File
_generate_module_nodejs(active_vision_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/active_vision_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(active_vision_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(active_vision_msgs_generate_messages active_vision_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pourya/Documents/Active_Perception/src/vision_msgs/msg/Vision_Message.msg" NAME_WE)
add_dependencies(active_vision_msgs_generate_messages_nodejs _active_vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pourya/Documents/Active_Perception/src/vision_msgs/srv/Vision_Service.srv" NAME_WE)
add_dependencies(active_vision_msgs_generate_messages_nodejs _active_vision_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(active_vision_msgs_gennodejs)
add_dependencies(active_vision_msgs_gennodejs active_vision_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS active_vision_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(active_vision_msgs
  "/home/pourya/Documents/Active_Perception/src/vision_msgs/msg/Vision_Message.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/active_vision_msgs
)

### Generating Services
_generate_srv_py(active_vision_msgs
  "/home/pourya/Documents/Active_Perception/src/vision_msgs/srv/Vision_Service.srv"
  "${MSG_I_FLAGS}"
  "/home/pourya/Documents/Active_Perception/src/vision_msgs/msg/Vision_Message.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/active_vision_msgs
)

### Generating Module File
_generate_module_py(active_vision_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/active_vision_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(active_vision_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(active_vision_msgs_generate_messages active_vision_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pourya/Documents/Active_Perception/src/vision_msgs/msg/Vision_Message.msg" NAME_WE)
add_dependencies(active_vision_msgs_generate_messages_py _active_vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pourya/Documents/Active_Perception/src/vision_msgs/srv/Vision_Service.srv" NAME_WE)
add_dependencies(active_vision_msgs_generate_messages_py _active_vision_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(active_vision_msgs_genpy)
add_dependencies(active_vision_msgs_genpy active_vision_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS active_vision_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/active_vision_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/active_vision_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(active_vision_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(active_vision_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/active_vision_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/active_vision_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(active_vision_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(active_vision_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/active_vision_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/active_vision_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(active_vision_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(active_vision_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/active_vision_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/active_vision_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(active_vision_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(active_vision_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/active_vision_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/active_vision_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/active_vision_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(active_vision_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(active_vision_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
