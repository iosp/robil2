cmake_minimum_required(VERSION 2.8.3)
project(oshkosh_model)

SET(CMAKE_CXX_FLAGS "-std=c++0x")


find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
  std_msgs
  geometry_msgs
  dynamic_reconfigure
)
find_package(Boost REQUIRED COMPONENTS 
   thread 
   system 
)

find_package(gazebo REQUIRED)

generate_dynamic_reconfigure_options(cfg/oshkosh_model.cfg)  


link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  roscpp
  std_msgs
  #dynamic_reconfigure
)


include_directories(  ${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(oshkosh_driving_plugin SHARED src/oshkosh_driving_plugin.cc)
target_link_libraries(oshkosh_driving_plugin ${catkin_LIBRARIES}  ${Boost_LIBRARIES} )
add_dependencies(oshkosh_driving_plugin ${PROJECT_NAME}_gencfg)