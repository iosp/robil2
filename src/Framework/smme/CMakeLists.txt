
cmake_minimum_required(VERSION 2.8.3)
project(smme)
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
add_library(smme_rosComm
  src/roscomm/RosComm.cpp
)
add_library(smme_compMain
  src/component/ComponentMain.cpp
  src/component/ComponentStates.cpp
  src/component/MissionStates.cpp
  src/component/TaskStates.cpp
  src/component/MissionManager.cpp
  src/component/AblManager.cpp
)
add_executable(smme_node src/main.cpp)
target_link_libraries(smme_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(smme_compMain ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(smme_node smme_compMain smme_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(fsm_component_tester src/tester/FsmComponentTester.cpp)
target_link_libraries(fsm_component_tester ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(mission_tester src/tester/MissionTester.cpp)
target_link_libraries(mission_tester ${catkin_LIBRARIES} ${Boost_LIBRARIES})







