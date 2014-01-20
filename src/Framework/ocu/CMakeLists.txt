cmake_minimum_required(VERSION 2.8.3)
project(ocu)

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



add_library(ocu_rosComm
  src/RosComm.cpp
)
add_library(ocu_compMain
  src/ComponentMain.cpp
)

add_executable(ocu_node src/main.cpp)

target_link_libraries(ocu_node ocu_compMain ocu_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

