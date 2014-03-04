cmake_minimum_required(VERSION 2.8.3)
project(gazebo_gps_ins)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(gazebo REQUIRED)


link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(gps_ins SHARED src/gps_ins.cpp)
target_link_libraries(gps_ins ${GAZEBO_libraries})

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   GPS_INS.msg
 )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

generate_messages()

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
    CATKIN_DEPENDS message_runtime
)


install(
  DIRECTORY models/
  DESTINATION $ENV{HOME}/.gazebo/models/GPS_INS
)
