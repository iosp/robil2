cmake_minimum_required(VERSION 2.8.3)
project(wsm)

find_package(catkin REQUIRED COMPONENTS
  robil2_msgs roscpp
)

find_package(Boost REQUIRED COMPONENTS system)



###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library

add_library(wsm_rosComm
  src/RosComm.cpp
)
add_library(wsm_compMain
  src/ComponentMain.cpp
)


add_executable(wsm_node src/main.cpp)

target_link_libraries(wsm_node wsm_compMain wsm_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

