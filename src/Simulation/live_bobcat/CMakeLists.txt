cmake_minimum_required(VERSION 2.8.3)
project(live_bobcat)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  std_msgs
  tf
  #message_generation
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(tf REQUIRED)
find_package(gazebo REQUIRED)


link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(TFpublisher SHARED src/TFpublisher.cpp)
target_link_libraries(TFpublisher ${GAZEBO_libraries} ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES live_bobcat
  CATKIN_DEPENDS 
       roscpp tf gazebo_ros bobcat gazebo_sick_gpu
#  DEPENDS system_lib
)




## Mark other files for installation (e.g. launch and bag files, etc.)
