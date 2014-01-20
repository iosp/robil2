cmake_minimum_required(VERSION 2.8.3)
project(lli)

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

add_library(lli_rosComm
  src/RosComm.cpp
)
add_library(lli_compMain
  src/ComponentMain.cpp
)


add_executable(lli_node src/main.cpp)

target_link_libraries(lli_node lli_compMain lli_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

