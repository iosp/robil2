# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "events_bus: 3 messages, 8 services")

set(MSG_I_FLAGS "-Ievents_bus:/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(events_bus_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_StatesMonitor_ActiveState.msg" NAME_WE)
add_custom_target(_events_bus_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "events_bus" "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_StatesMonitor_ActiveState.msg" ""
)

get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Result.srv" NAME_WE)
add_custom_target(_events_bus_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "events_bus" "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Result.srv" ""
)

get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_StatesMonitor_StateResult.msg" NAME_WE)
add_custom_target(_events_bus_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "events_bus" "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_StatesMonitor_StateResult.msg" ""
)

get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_Event.msg" NAME_WE)
add_custom_target(_events_bus_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "events_bus" "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_Event.msg" ""
)

get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_States.srv" NAME_WE)
add_custom_target(_events_bus_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "events_bus" "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_States.srv" ""
)

get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_RawStatus.srv" NAME_WE)
add_custom_target(_events_bus_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "events_bus" "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_RawStatus.srv" "events_bus/Msg_StatesMonitor_ActiveState:events_bus/Msg_StatesMonitor_StateResult"
)

get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Status.srv" NAME_WE)
add_custom_target(_events_bus_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "events_bus" "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Status.srv" ""
)

get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Is.srv" NAME_WE)
add_custom_target(_events_bus_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "events_bus" "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Is.srv" ""
)

get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_EndStates.srv" NAME_WE)
add_custom_target(_events_bus_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "events_bus" "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_EndStates.srv" ""
)

get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Times.srv" NAME_WE)
add_custom_target(_events_bus_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "events_bus" "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Times.srv" ""
)

get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_Version.srv" NAME_WE)
add_custom_target(_events_bus_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "events_bus" "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_Version.srv" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_StatesMonitor_ActiveState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/events_bus
)
_generate_msg_cpp(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_StatesMonitor_StateResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/events_bus
)
_generate_msg_cpp(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_Event.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/events_bus
)

### Generating Services
_generate_srv_cpp(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Result.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/events_bus
)
_generate_srv_cpp(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_States.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/events_bus
)
_generate_srv_cpp(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_RawStatus.srv"
  "${MSG_I_FLAGS}"
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_StatesMonitor_ActiveState.msg;/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_StatesMonitor_StateResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/events_bus
)
_generate_srv_cpp(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_EndStates.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/events_bus
)
_generate_srv_cpp(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Is.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/events_bus
)
_generate_srv_cpp(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Status.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/events_bus
)
_generate_srv_cpp(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Times.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/events_bus
)
_generate_srv_cpp(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_Version.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/events_bus
)

### Generating Module File
_generate_module_cpp(events_bus
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/events_bus
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(events_bus_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(events_bus_generate_messages events_bus_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_StatesMonitor_ActiveState.msg" NAME_WE)
add_dependencies(events_bus_generate_messages_cpp _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Result.srv" NAME_WE)
add_dependencies(events_bus_generate_messages_cpp _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_StatesMonitor_StateResult.msg" NAME_WE)
add_dependencies(events_bus_generate_messages_cpp _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_Event.msg" NAME_WE)
add_dependencies(events_bus_generate_messages_cpp _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_States.srv" NAME_WE)
add_dependencies(events_bus_generate_messages_cpp _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_RawStatus.srv" NAME_WE)
add_dependencies(events_bus_generate_messages_cpp _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Status.srv" NAME_WE)
add_dependencies(events_bus_generate_messages_cpp _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Is.srv" NAME_WE)
add_dependencies(events_bus_generate_messages_cpp _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_EndStates.srv" NAME_WE)
add_dependencies(events_bus_generate_messages_cpp _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Times.srv" NAME_WE)
add_dependencies(events_bus_generate_messages_cpp _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_Version.srv" NAME_WE)
add_dependencies(events_bus_generate_messages_cpp _events_bus_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(events_bus_gencpp)
add_dependencies(events_bus_gencpp events_bus_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS events_bus_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_StatesMonitor_ActiveState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/events_bus
)
_generate_msg_lisp(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_StatesMonitor_StateResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/events_bus
)
_generate_msg_lisp(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_Event.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/events_bus
)

### Generating Services
_generate_srv_lisp(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Result.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/events_bus
)
_generate_srv_lisp(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_States.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/events_bus
)
_generate_srv_lisp(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_RawStatus.srv"
  "${MSG_I_FLAGS}"
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_StatesMonitor_ActiveState.msg;/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_StatesMonitor_StateResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/events_bus
)
_generate_srv_lisp(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_EndStates.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/events_bus
)
_generate_srv_lisp(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Is.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/events_bus
)
_generate_srv_lisp(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Status.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/events_bus
)
_generate_srv_lisp(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Times.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/events_bus
)
_generate_srv_lisp(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_Version.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/events_bus
)

### Generating Module File
_generate_module_lisp(events_bus
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/events_bus
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(events_bus_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(events_bus_generate_messages events_bus_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_StatesMonitor_ActiveState.msg" NAME_WE)
add_dependencies(events_bus_generate_messages_lisp _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Result.srv" NAME_WE)
add_dependencies(events_bus_generate_messages_lisp _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_StatesMonitor_StateResult.msg" NAME_WE)
add_dependencies(events_bus_generate_messages_lisp _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_Event.msg" NAME_WE)
add_dependencies(events_bus_generate_messages_lisp _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_States.srv" NAME_WE)
add_dependencies(events_bus_generate_messages_lisp _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_RawStatus.srv" NAME_WE)
add_dependencies(events_bus_generate_messages_lisp _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Status.srv" NAME_WE)
add_dependencies(events_bus_generate_messages_lisp _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Is.srv" NAME_WE)
add_dependencies(events_bus_generate_messages_lisp _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_EndStates.srv" NAME_WE)
add_dependencies(events_bus_generate_messages_lisp _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Times.srv" NAME_WE)
add_dependencies(events_bus_generate_messages_lisp _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_Version.srv" NAME_WE)
add_dependencies(events_bus_generate_messages_lisp _events_bus_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(events_bus_genlisp)
add_dependencies(events_bus_genlisp events_bus_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS events_bus_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_StatesMonitor_ActiveState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/events_bus
)
_generate_msg_py(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_StatesMonitor_StateResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/events_bus
)
_generate_msg_py(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_Event.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/events_bus
)

### Generating Services
_generate_srv_py(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Result.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/events_bus
)
_generate_srv_py(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_States.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/events_bus
)
_generate_srv_py(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_RawStatus.srv"
  "${MSG_I_FLAGS}"
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_StatesMonitor_ActiveState.msg;/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_StatesMonitor_StateResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/events_bus
)
_generate_srv_py(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_EndStates.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/events_bus
)
_generate_srv_py(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Is.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/events_bus
)
_generate_srv_py(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Status.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/events_bus
)
_generate_srv_py(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Times.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/events_bus
)
_generate_srv_py(events_bus
  "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_Version.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/events_bus
)

### Generating Module File
_generate_module_py(events_bus
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/events_bus
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(events_bus_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(events_bus_generate_messages events_bus_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_StatesMonitor_ActiveState.msg" NAME_WE)
add_dependencies(events_bus_generate_messages_py _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Result.srv" NAME_WE)
add_dependencies(events_bus_generate_messages_py _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_StatesMonitor_StateResult.msg" NAME_WE)
add_dependencies(events_bus_generate_messages_py _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/msg/Msg_Event.msg" NAME_WE)
add_dependencies(events_bus_generate_messages_py _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_States.srv" NAME_WE)
add_dependencies(events_bus_generate_messages_py _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_RawStatus.srv" NAME_WE)
add_dependencies(events_bus_generate_messages_py _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Status.srv" NAME_WE)
add_dependencies(events_bus_generate_messages_py _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Is.srv" NAME_WE)
add_dependencies(events_bus_generate_messages_py _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_EndStates.srv" NAME_WE)
add_dependencies(events_bus_generate_messages_py _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_StatesMonitor_Times.srv" NAME_WE)
add_dependencies(events_bus_generate_messages_py _events_bus_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/assaf/workspaces/robil_workspace/robil2/src/Framework/cognitao/events_bus/srv/Srv_Version.srv" NAME_WE)
add_dependencies(events_bus_generate_messages_py _events_bus_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(events_bus_genpy)
add_dependencies(events_bus_genpy events_bus_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS events_bus_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/events_bus)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/events_bus
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(events_bus_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/events_bus)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/events_bus
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(events_bus_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/events_bus)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/events_bus\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/events_bus
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(events_bus_generate_messages_py std_msgs_generate_messages_py)
