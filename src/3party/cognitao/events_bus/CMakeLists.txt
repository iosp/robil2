cmake_minimum_required(VERSION 2.8.3)
project(events_bus)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)


add_message_files(
  FILES
    Msg_Event.msg
    
    Msg_StatesMonitor_ActiveState.msg
    Msg_StatesMonitor_StateResult.msg
)

add_service_files(
  FILES
    Srv_StatesMonitor_EndStates.srv
    Srv_StatesMonitor_Is.srv
    Srv_StatesMonitor_Result.srv
    Srv_StatesMonitor_States.srv
    Srv_StatesMonitor_Status.srv
    Srv_StatesMonitor_Times.srv
    Srv_StatesMonitor_RawStatus.srv
    
    Srv_Version.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES events_bus ros_events_bus
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

set( EVENTS_CPP
    src/HierarchicalName.cpp
    src/Event.cpp
    src/EventQueue.cpp
    src/StatesMonitor.cpp
)
set( ROS_EVENTS_CPP
    src/ros_adapter/RosEventQueue.cpp
    src/ros_adapter/RosStatesMonitor.cpp
    src/ros_adapter/Heart.cpp
)
set( EXAMPLE_CPP
    src/node/example.cpp
)
set( STATES_MONITOR_NODE_CPP
    src/node/states_monitor_node.cpp
    src/node/heartbeat_monitor_node.cpp
)
set( HEART_NODE_CPP
    src/node/heartbeat_monitor_node.cpp
)

#--------- LIBS ----------------------------------------------
add_library(events_bus ${EVENTS_CPP} )
target_link_libraries(events_bus  ${catkin_LIBRARIES} ${Boost_LIBRARIES} )

add_library(ros_events_bus ${ROS_EVENTS_CPP} )
add_dependencies(ros_events_bus events_bus_generate_messages_cpp)
target_link_libraries(ros_events_bus  
    ${catkin_LIBRARIES} ${Boost_LIBRARIES} 
    events_bus
)

#--------- BIN -----------------------------------------------
add_executable(events_bus_example_node ${EXAMPLE_CPP})
target_link_libraries(events_bus_example_node
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} 
  events_bus
  ros_events_bus
)

add_executable(states_monitor_node ${STATES_MONITOR_NODE_CPP})
target_link_libraries(states_monitor_node
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} 
  events_bus
  ros_events_bus
)

#add_executable(heartbeat_monitor_node ${HEART_NODE_CPP})
#target_link_libraries(heartbeat_monitor_node
#  ${catkin_LIBRARIES} ${Boost_LIBRARIES} 
#  events_bus
#  ros_events_bus
#)


add_executable(events_hub_node src/node/EventsHub.cpp)
target_link_libraries(events_hub_node
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} 
  events_bus
  ros_events_bus
)

add_executable(states_monitor_aggregator_node src/node/states_monitor_aggregator_node.cpp src/ros_adapter/RosStatesMonitorAggregator.cpp)
target_link_libraries(states_monitor_aggregator_node
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} 
  events_bus
  ros_events_bus
)


#--------- TESTS -----------------------------------------------

#add_executable(test_HierarchicalName
#   
#   src/HierarchicalName.cpp
#
#   src/node/tests/test_HierarchicalName.cpp
#)
#target_link_libraries(test_HierarchicalName
#  ${catkin_LIBRARIES} ${Boost_LIBRARIES} 
#)


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
install(TARGETS events_bus_example_node states_monitor_node events_bus ros_events_bus events_hub_node states_monitor_aggregator_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  #FILES_MATCHING PATTERN "*.h"
  #PATTERN ".svn" EXCLUDE
)

#install(DIRECTORY include/events_bus/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  #FILES_MATCHING PATTERN "*.h"
#  #PATTERN ".svn" EXCLUDE
#)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )




