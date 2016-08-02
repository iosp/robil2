
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(ROS_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Debug)

cmake_minimum_required(VERSION 2.8.3)
project(shiffon2ros)
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
#add_library(shiffon2ros_rosComm
#  src/roscomm/RosComm.cpp
#)
add_library(shiffon2ros_Ctrl
  src/Shiphon_IO/Shiphon_IO.cpp
  src/Shiphon_IO/ShareUtil.cpp
)

add_library(shiffon2ros_compMain
  src/component/ComponentMain.cpp
  src/component/ComponentStates.cpp
)
add_executable(shiffon2ros_node src/main.cpp)
#target_link_libraries(shiffon2ros_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(shiffon2ros_compMain ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(shiffon2ros_node shiffon2ros_compMain shiffon2ros_Ctrl ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

