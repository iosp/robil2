cmake_minimum_required(VERSION 2.8.3)
project(se_r2rs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs roscpp
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

add_library(se_r2rs_rosComm
  src/RosComm.cpp
)
add_library(se_r2rs_compMain
  src/ComponentMain.cpp
)

add_executable(se_r2rs_node src/main.cpp)

target_link_libraries(se_r2rs_node se_r2rs_compMain se_r2rs_rosComm  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

