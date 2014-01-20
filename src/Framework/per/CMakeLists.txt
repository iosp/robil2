cmake_minimum_required(VERSION 2.8.3)
project(per)

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

add_library(per_rosComm
  src/RosComm.cpp
)
add_library(per_compMain
  src/ComponentMain.cpp
)


add_executable(per_node src/main.cpp)

target_link_libraries(per_node per_compMain per_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

