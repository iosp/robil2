cmake_minimum_required(VERSION 2.8.3)
project(gazebo_wire_sensor)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(gazebo REQUIRED)


link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(wire_sensor SHARED src/wire_sensor.cpp)
target_link_libraries(wire_sensor ${GAZEBO_libraries})


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )
catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
    CATKIN_DEPENDS message_runtime
)

