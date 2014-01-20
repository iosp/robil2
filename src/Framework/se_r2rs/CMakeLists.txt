cmake_minimum_required(VERSION 2.8.3)
project(se_r2rs)

find_package(catkin REQUIRED COMPONENTS
  robil2_msgs roscpp
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(

)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library

add_library(ser2rs_rosComm
  src/RosComm.cpp
)
add_library(ser2rs_compMain
  src/ComponentMain.cpp
)

add_executable(ser2rs_node src/main.cpp)

target_link_libraries(ser2rs_node ser2rs_compMain ser2rs_rosComm  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

