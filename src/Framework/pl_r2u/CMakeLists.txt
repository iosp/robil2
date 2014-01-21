cmake_minimum_required(VERSION 2.8.3)
project(pl_r2u)

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

add_library(pl_r2u_rosComm
  src/RosComm.cpp
)
add_library(pl_r2u_compMain
  src/ComponentMain.cpp
)


add_executable(pl_r2u_node src/main.cpp)

target_link_libraries(pl_r2u_node pl_r2u_compMain pl_r2u_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

