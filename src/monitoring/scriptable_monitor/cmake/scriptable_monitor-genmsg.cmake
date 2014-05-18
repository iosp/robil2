# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "scriptable_monitor: 1 messages, 2 services")

set(MSG_I_FLAGS "-Iscriptable_monitor:/home/dan/git/scriptable_monitoring/scriptable_monitor/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(scriptable_monitor_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(scriptable_monitor
  "/home/dan/git/scriptable_monitoring/scriptable_monitor/msg/Script.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/scriptable_monitor
)

### Generating Services
_generate_srv_cpp(scriptable_monitor
  "/home/dan/git/scriptable_monitoring/scriptable_monitor/srv/AddScript.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/scriptable_monitor
)
_generate_srv_cpp(scriptable_monitor
  "/home/dan/git/scriptable_monitoring/scriptable_monitor/srv/GetScripts.srv"
  "${MSG_I_FLAGS}"
  "/home/dan/git/scriptable_monitoring/scriptable_monitor/msg/Script.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/scriptable_monitor
)

### Generating Module File
_generate_module_cpp(scriptable_monitor
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/scriptable_monitor
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(scriptable_monitor_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(scriptable_monitor_generate_messages scriptable_monitor_generate_messages_cpp)

# target for backward compatibility
add_custom_target(scriptable_monitor_gencpp)
add_dependencies(scriptable_monitor_gencpp scriptable_monitor_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS scriptable_monitor_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(scriptable_monitor
  "/home/dan/git/scriptable_monitoring/scriptable_monitor/msg/Script.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/scriptable_monitor
)

### Generating Services
_generate_srv_lisp(scriptable_monitor
  "/home/dan/git/scriptable_monitoring/scriptable_monitor/srv/AddScript.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/scriptable_monitor
)
_generate_srv_lisp(scriptable_monitor
  "/home/dan/git/scriptable_monitoring/scriptable_monitor/srv/GetScripts.srv"
  "${MSG_I_FLAGS}"
  "/home/dan/git/scriptable_monitoring/scriptable_monitor/msg/Script.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/scriptable_monitor
)

### Generating Module File
_generate_module_lisp(scriptable_monitor
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/scriptable_monitor
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(scriptable_monitor_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(scriptable_monitor_generate_messages scriptable_monitor_generate_messages_lisp)

# target for backward compatibility
add_custom_target(scriptable_monitor_genlisp)
add_dependencies(scriptable_monitor_genlisp scriptable_monitor_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS scriptable_monitor_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(scriptable_monitor
  "/home/dan/git/scriptable_monitoring/scriptable_monitor/msg/Script.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/scriptable_monitor
)

### Generating Services
_generate_srv_py(scriptable_monitor
  "/home/dan/git/scriptable_monitoring/scriptable_monitor/srv/AddScript.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/scriptable_monitor
)
_generate_srv_py(scriptable_monitor
  "/home/dan/git/scriptable_monitoring/scriptable_monitor/srv/GetScripts.srv"
  "${MSG_I_FLAGS}"
  "/home/dan/git/scriptable_monitoring/scriptable_monitor/msg/Script.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/scriptable_monitor
)

### Generating Module File
_generate_module_py(scriptable_monitor
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/scriptable_monitor
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(scriptable_monitor_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(scriptable_monitor_generate_messages scriptable_monitor_generate_messages_py)

# target for backward compatibility
add_custom_target(scriptable_monitor_genpy)
add_dependencies(scriptable_monitor_genpy scriptable_monitor_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS scriptable_monitor_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/scriptable_monitor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/scriptable_monitor
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(scriptable_monitor_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/scriptable_monitor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/scriptable_monitor
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(scriptable_monitor_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/scriptable_monitor)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/scriptable_monitor\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/scriptable_monitor
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(scriptable_monitor_generate_messages_py std_msgs_generate_messages_py)
