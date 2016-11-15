
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(ROS_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Debug)

cmake_minimum_required(VERSION 2.8.3)
project(shiffon2ros)
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
<<<<<<< HEAD
#target_link_libraries(shiffon2ros_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(shiffon2ros_compMain ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(shiffon2ros_node shiffon2ros_compMain shiffon2ros_Ctrl ${catkin_LIBRARIES} ${Boost_LIBRARIES}
=======
target_link_libraries(shiffon2ros_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES} cognitao_v2)
target_link_libraries(shiffon2ros_compMain ${catkin_LIBRARIES} ${Boost_LIBRARIES} cognitao_v2)
target_link_libraries(shiffon2ros_node shiffon2ros_compMain shiffon2ros_rosComm shiffon2ros_Ctrl ${catkin_LIBRARIES} ${Boost_LIBRARIES} cognitao_v2
>>>>>>> origin/moving_to_new_cognitao
)

