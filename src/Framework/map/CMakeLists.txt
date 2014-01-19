cmake_minimum_required(VERSION 2.8.3)
project(map)

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

add_library(map_rosComm
  src/RosComm.cpp
)
add_library(map_compMain
  src/ComponentMain.cpp
)

add_executable(map_node src/main.cpp)

target_link_libraries(map_node map_compMain map_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

