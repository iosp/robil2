cmake_minimum_required(VERSION 2.8.3)
project(gazebo_camera)


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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(cameraTF SHARED src/cameraTF.cpp)
target_link_libraries(cameraTF ${GAZEBO_libraries} ${catkin_LIBRARIES})

## Generate messages in the 'msg' folder
 #add_message_files(
 #  FILES
 #  LaserScan.msg
#   Message2.msg
 #)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )


catkin_package(
  DEPENDS 
    roscpp 
    tf
    gazebo_ros
    CATKIN_DEPENDS message_runtime
)

