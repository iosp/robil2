cmake_minimum_required(VERSION 2.8.3)
project(ipon)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(gazebo REQUIRED)


link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(IPON SHARED src/IPON.cpp)
target_link_libraries(IPON ${GAZEBO_libraries})


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")


catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros
    CATKIN_DEPENDS message_runtime  
)

