
cmake_minimum_required(VERSION 2.8.3)
project(ocu)
find_package(catkin REQUIRED COMPONENTS
    roscpp 
	rospy
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
#add_library(ocu_rosComm
#  src/roscomm/RosComm.cpp
#)
add_library(ocu_compMain
  src/component/ComponentMain.cpp
  src/component/ComponentStates.cpp
)
add_executable(ocu_node src/main.cpp)
#target_link_libraries(ocu_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(ocu_compMain ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(ocu_node ocu_compMain  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

install(PROGRAMS
   scripts/ocu.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

