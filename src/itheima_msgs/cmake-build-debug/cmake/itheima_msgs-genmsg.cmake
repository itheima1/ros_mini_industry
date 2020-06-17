# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "itheima_msgs: 1 messages, 1 services")

set(MSG_I_FLAGS "-Iitheima_msgs:/home/itcast/rosws/itheimalab_ws/src/itheima_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(itheima_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/itcast/rosws/itheimalab_ws/src/itheima_msgs/srv/AssemblyLineCtrl.srv" NAME_WE)
add_custom_target(_itheima_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "itheima_msgs" "/home/itcast/rosws/itheimalab_ws/src/itheima_msgs/srv/AssemblyLineCtrl.srv" ""
)

get_filename_component(_filename "/home/itcast/rosws/itheimalab_ws/src/itheima_msgs/msg/AssemblyIR.msg" NAME_WE)
add_custom_target(_itheima_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "itheima_msgs" "/home/itcast/rosws/itheimalab_ws/src/itheima_msgs/msg/AssemblyIR.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(itheima_msgs
  "/home/itcast/rosws/itheimalab_ws/src/itheima_msgs/msg/AssemblyIR.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itheima_msgs
)

### Generating Services
_generate_srv_cpp(itheima_msgs
  "/home/itcast/rosws/itheimalab_ws/src/itheima_msgs/srv/AssemblyLineCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itheima_msgs
)

### Generating Module File
_generate_module_cpp(itheima_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itheima_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(itheima_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(itheima_msgs_generate_messages itheima_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/itcast/rosws/itheimalab_ws/src/itheima_msgs/srv/AssemblyLineCtrl.srv" NAME_WE)
add_dependencies(itheima_msgs_generate_messages_cpp _itheima_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itcast/rosws/itheimalab_ws/src/itheima_msgs/msg/AssemblyIR.msg" NAME_WE)
add_dependencies(itheima_msgs_generate_messages_cpp _itheima_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(itheima_msgs_gencpp)
add_dependencies(itheima_msgs_gencpp itheima_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS itheima_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(itheima_msgs
  "/home/itcast/rosws/itheimalab_ws/src/itheima_msgs/msg/AssemblyIR.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itheima_msgs
)

### Generating Services
_generate_srv_eus(itheima_msgs
  "/home/itcast/rosws/itheimalab_ws/src/itheima_msgs/srv/AssemblyLineCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itheima_msgs
)

### Generating Module File
_generate_module_eus(itheima_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itheima_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(itheima_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(itheima_msgs_generate_messages itheima_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/itcast/rosws/itheimalab_ws/src/itheima_msgs/srv/AssemblyLineCtrl.srv" NAME_WE)
add_dependencies(itheima_msgs_generate_messages_eus _itheima_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itcast/rosws/itheimalab_ws/src/itheima_msgs/msg/AssemblyIR.msg" NAME_WE)
add_dependencies(itheima_msgs_generate_messages_eus _itheima_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(itheima_msgs_geneus)
add_dependencies(itheima_msgs_geneus itheima_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS itheima_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(itheima_msgs
  "/home/itcast/rosws/itheimalab_ws/src/itheima_msgs/msg/AssemblyIR.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itheima_msgs
)

### Generating Services
_generate_srv_lisp(itheima_msgs
  "/home/itcast/rosws/itheimalab_ws/src/itheima_msgs/srv/AssemblyLineCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itheima_msgs
)

### Generating Module File
_generate_module_lisp(itheima_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itheima_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(itheima_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(itheima_msgs_generate_messages itheima_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/itcast/rosws/itheimalab_ws/src/itheima_msgs/srv/AssemblyLineCtrl.srv" NAME_WE)
add_dependencies(itheima_msgs_generate_messages_lisp _itheima_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itcast/rosws/itheimalab_ws/src/itheima_msgs/msg/AssemblyIR.msg" NAME_WE)
add_dependencies(itheima_msgs_generate_messages_lisp _itheima_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(itheima_msgs_genlisp)
add_dependencies(itheima_msgs_genlisp itheima_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS itheima_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(itheima_msgs
  "/home/itcast/rosws/itheimalab_ws/src/itheima_msgs/msg/AssemblyIR.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itheima_msgs
)

### Generating Services
_generate_srv_nodejs(itheima_msgs
  "/home/itcast/rosws/itheimalab_ws/src/itheima_msgs/srv/AssemblyLineCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itheima_msgs
)

### Generating Module File
_generate_module_nodejs(itheima_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itheima_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(itheima_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(itheima_msgs_generate_messages itheima_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/itcast/rosws/itheimalab_ws/src/itheima_msgs/srv/AssemblyLineCtrl.srv" NAME_WE)
add_dependencies(itheima_msgs_generate_messages_nodejs _itheima_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itcast/rosws/itheimalab_ws/src/itheima_msgs/msg/AssemblyIR.msg" NAME_WE)
add_dependencies(itheima_msgs_generate_messages_nodejs _itheima_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(itheima_msgs_gennodejs)
add_dependencies(itheima_msgs_gennodejs itheima_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS itheima_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(itheima_msgs
  "/home/itcast/rosws/itheimalab_ws/src/itheima_msgs/msg/AssemblyIR.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itheima_msgs
)

### Generating Services
_generate_srv_py(itheima_msgs
  "/home/itcast/rosws/itheimalab_ws/src/itheima_msgs/srv/AssemblyLineCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itheima_msgs
)

### Generating Module File
_generate_module_py(itheima_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itheima_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(itheima_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(itheima_msgs_generate_messages itheima_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/itcast/rosws/itheimalab_ws/src/itheima_msgs/srv/AssemblyLineCtrl.srv" NAME_WE)
add_dependencies(itheima_msgs_generate_messages_py _itheima_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itcast/rosws/itheimalab_ws/src/itheima_msgs/msg/AssemblyIR.msg" NAME_WE)
add_dependencies(itheima_msgs_generate_messages_py _itheima_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(itheima_msgs_genpy)
add_dependencies(itheima_msgs_genpy itheima_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS itheima_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itheima_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itheima_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(itheima_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itheima_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itheima_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(itheima_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itheima_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itheima_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(itheima_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itheima_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itheima_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(itheima_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itheima_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itheima_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itheima_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(itheima_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
