
cmake_minimum_required(VERSION 2.8.3)
project(ssm)
find_package(catkin REQUIRED COMPONENTS
    roscpp rospy
    robil2conf
    decision_making decision_making_parser
)
find_package(Boost REQUIRED COMPONENTS system)
catkin_package(
	CATKIN_DEPENDS
	    robil2conf
	    decision_making decision_making_parser
)
###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
## Declare a cpp library
add_library(ssm_rosComm
  src/roscomm/RosComm.cpp
)
add_library(ssm_compMain
  src/component/ComponentMain.cpp
  src/component/ComponentStates.cpp
)
add_executable(ssm_node src/main.cpp)
target_link_libraries(ssm_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(ssm_compMain ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(ssm_node ssm_compMain ssm_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(ssm_heartbeat_monitor_node src/monitor/HeartBeatMonitor.cpp)
target_link_libraries(ssm_heartbeat_monitor_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(ssm_fsm_states_tracker_node src/monitor/FsmStatesTracker.cpp)
target_link_libraries(ssm_fsm_states_tracker_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
