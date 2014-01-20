cmake_minimum_required(VERSION 2.8.3)
project(smme)

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

add_library(smme_rosComm
  src/RosComm.cpp
)
add_library(smme_compMain
  src/ComponentMain.cpp
)


add_executable(smme_node src/main.cpp)

target_link_libraries(smme_node smme_compMain smme_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

