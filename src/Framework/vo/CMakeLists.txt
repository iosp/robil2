cmake_minimum_required(VERSION 2.8.3)
project(vo)

find_package(catkin REQUIRED COMPONENTS
 roscpp rospy std_msgs robil2conf
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
	CATKIN_DEPENDS std_msgs robil2conf
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library

add_library(vo_rosComm
  src/roscomm/RosComm.cpp
)
add_library(vo_compMain
  src/component/ComponentMain.cpp
)


add_executable(vo_node src/main.cpp)

target_link_libraries(vo_node vo_compMain vo_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

