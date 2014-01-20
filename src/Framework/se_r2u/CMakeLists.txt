cmake_minimum_required(VERSION 2.8.3)
project(se_r2u)

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

add_library(se_r2u_rosComm
  src/RosComm.cpp
)
add_library(se_r2u_compMain
  src/ComponentMain.cpp
)

add_executable(se_r2u_node src/main.cpp)

target_link_libraries(se_r2u_node se_r2u_compMain se_r2u_rosComm  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

