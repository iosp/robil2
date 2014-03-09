cmake_minimum_required(VERSION 2.8.3)
project(gazebo_sick_gpu)


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

add_library(sick_gpu SHARED src/sick.cpp)
target_link_libraries(sick_gpu ${GAZEBO_libraries} ${catkin_LIBRARIES})

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
   # CATKIN_DEPENDS message_runtime
)

install(
  DIRECTORY models/
  DESTINATION $ENV{HOME}/.gazebo/models/sick
)
