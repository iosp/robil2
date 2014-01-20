cmake_minimum_required(VERSION 2.8.3)
project(loc)

find_package(catkin REQUIRED COMPONENTS
 roscpp std_msgs
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

add_library(loc_rosComm
  src/RosComm.cpp
)
add_library(loc_compMain
  src/ComponentMain.cpp
)

add_executable(loc_node src/main.cpp)

target_link_libraries(loc_node loc_compMain loc_rosComm  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

