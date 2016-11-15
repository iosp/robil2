cmake_minimum_required(VERSION 2.8.3)
project(robil2tf)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(ROS_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Debug)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(tf REQUIRED)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(robil2TF_node src/robil2TF_node.cpp)
target_link_libraries(robil2TF_node
   ${catkin_LIBRARIES}
 )
 

