
cmake_minimum_required(VERSION 2.8.3)
project(ssm)
find_package(catkin REQUIRED COMPONENTS
    roscpp rospy
    robil2conf
    cognitao_io
    cognitao
    events_bus
    cognitao_event_bus_converter
    cognitao_v2
)
find_package(Boost REQUIRED COMPONENTS system)
catkin_package(
	CATKIN_DEPENDS
	    robil2conf
	    cognitao_event_bus_converter
)
###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${cognitao_io_INCLUDE_DIRS}
  ${cognitao_INCLUDE_DIRS}
  ${events_bus_INCLUDE_DIRS}
  ${cognitao_v2_INCLUDE_DIRS}
)
## Declare a cpp library
#add_library(ssm_rosComm
#  src/roscomm/RosComm.cpp
#)
add_library(ssm_compMain
  src/component/ComponentMain.cpp
  src/component/ComponentStates.cpp
)
add_executable(ssm_node src/main.cpp)
<<<<<<< HEAD
#target_link_libraries(ssm_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(ssm_compMain ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(ssm_node ssm_compMain  ${catkin_LIBRARIES} ${Boost_LIBRARIES})
=======
target_link_libraries(ssm_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES} cognitao_v2)
target_link_libraries(ssm_compMain ${catkin_LIBRARIES} ${Boost_LIBRARIES} cognitao_v2)
target_link_libraries(ssm_node ssm_compMain ssm_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES} cognitao_v2)
>>>>>>> origin/moving_to_new_cognitao

add_executable(ssm_heartbeat_monitor_node src/monitor/HeartBeatMonitor.cpp)
target_link_libraries(ssm_heartbeat_monitor_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} cognitao_v2)

add_executable(ssm_fsm_states_tracker_node src/monitor/FsmStatesTracker.cpp)
target_link_libraries(ssm_fsm_states_tracker_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} cognitao_v2)
