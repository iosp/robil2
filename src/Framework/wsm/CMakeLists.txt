
cmake_minimum_required(VERSION 2.8.3)
project(wsm)
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
#add_library(wsm_rosComm
#  src/roscomm/RosComm.cpp
#)
add_library(wsm_compMain
  src/component/ComponentMain.cpp
  src/component/ComponentStates.cpp
  src/component/InverseKinematics.cpp
  src/component/WsmTask.cpp
)
add_executable(wsm_node src/main.cpp)
#target_link_libraries(wsm_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(wsm_compMain ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(wsm_node wsm_compMain  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

