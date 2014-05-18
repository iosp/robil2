cmake_minimum_required(VERSION 2.8.3)
project(scriptable_monitor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  diagnostic_aggregator
  diagnostic_msgs
  pluginlib
  roscpp
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
    FILES
    Script.msg
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    GetScripts.srv
    AddScript.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS message_runtime diagnostic_aggregator diagnostic_msgs pluginlib roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
set(CPP_PLP_FILES 
	src/scripting/plp/PLPCompiler.cpp
	src/scripting/plp/PLPParser.cpp
)
set(CPP_FILES 
    src/scriptable_monitor.cpp
	src/scripting/Parameters.cpp
	src/scripting/InternalFunction.cpp
	src/scripting/PredicatesScript.cpp
	src/scripting/ScriptExecuter.cpp
	src/scripting/PythonExecuter.cpp
	src/scripting/RosTopicListener.cpp
	src/scripting/ScriptHost.cpp
	src/scripting/ScriptableMonitorNode.cpp
	src/scripting/SomeInternalFunctionExample.cpp
	
	src/scripting/ScriptParameters.cpp
	${CPP_PLP_FILES}
)

add_library(${PROJECT_NAME} ${CPP_FILES})
add_executable(monitor_test src/main.cpp ${CPP_FILES})
add_executable(scriptable_monitor_node src/scriptable_monitor_node.cpp ${CPP_FILES})
add_executable(plp_test src/scripting/plp/main.cpp ${CPP_PLP_FILES})


add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
add_dependencies(monitor_test ${PROJECT_NAME}_gencpp)
add_dependencies(scriptable_monitor_node ${PROJECT_NAME}_gencpp)
add_dependencies(plp_test ${PROJECT_NAME}_gencpp)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  python2.7
  yaml-cpp
)

target_link_libraries(monitor_test
  ${catkin_LIBRARIES}
  python2.7
  yaml-cpp
)

target_link_libraries(scriptable_monitor_node
  ${catkin_LIBRARIES}
  python2.7
  yaml-cpp
)

target_link_libraries(plp_test
  ${catkin_LIBRARIES}
  python2.7
  yaml-cpp
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS scriptable_analyzer scriptable_analyzer_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_scriptable_analyzer.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
