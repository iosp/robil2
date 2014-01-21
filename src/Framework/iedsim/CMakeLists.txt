cmake_minimum_required(VERSION 2.8.3)
project(iedsim)

find_package(catkin REQUIRED COMPONENTS
 roscpp rospy std_msgs genmsg
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
	CATKIN_DEPENDS std_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library

add_library(iedsim_rosComm
  src/RosComm.cpp
)
add_library(iedsim_compMain
  src/ComponentMain.cpp
)

add_executable(iedsim_node src/main.cpp)

target_link_libraries(iedsim_node iedsim_compMain iedsim_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)


