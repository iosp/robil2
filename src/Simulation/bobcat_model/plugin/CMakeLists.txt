cmake_minimum_required(VERSION 2.8.3)
project(bobtank_plugin)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  std_msgs
)
find_package(Boost REQUIRED COMPONENTS 
   thread 
   system 
)
find_package(gazebo REQUIRED)



link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(bobtank_drive_plugin SHARED src/bobtank_drive_plugin.cc)
target_link_libraries(bobtank_drive_plugin 
  ${catkin_LIBRARIES}  
  ${Boost_LIBRARIES} 	
		)


catkin_package()

include_directories(  ${catkin_INCLUDE_DIRS})
