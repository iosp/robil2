
cmake_minimum_required(VERSION 2.8.3)
project(lli)
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
add_library(lli_rosComm
  src/roscomm/RosComm.cpp
)
add_library(LLICtrl 
  src/QinetiQ_IO/LLICtrl.cpp
  src/QinetiQ_IO/ShareUtil.cpp 
)
add_library(lli_compMain
  src/component/ComponentMain.cpp
  src/component/ComponentStates.cpp
 ## src/component/lliCtrlManager.cpp
)

add_executable(lli_node src/main.cpp)
target_link_libraries(lli_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(lli_compMain ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(lli_node lli_compMain lli_rosComm LLICtrl ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)


