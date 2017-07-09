cmake_minimum_required(VERSION 2.8.3)
project(bobcat_model)

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

generate_dynamic_reconfigure_options(
       cfg/bobcat_model.cfg  
       cfg/bobcat_tracked_model.cfg
       cfg/bobcat_arm.cfg
)  

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


add_library(bobtank_drive_plugin SHARED src/bobtank_drive_plugin.cc)
target_link_libraries(bobtank_drive_plugin ${catkin_LIBRARIES}  ${Boost_LIBRARIES} )


add_library(bobcat_drive_plugin SHARED src/bobcat_drive_plugin.cc)
target_link_libraries(bobcat_drive_plugin ${catkin_LIBRARIES}  ${Boost_LIBRARIES} )
add_dependencies(bobcat_drive_plugin ${PROJECT_NAME}_gencfg)

add_library(bobcat_tracked_drive_plugin SHARED src/bobcat_tracked_drive_plugin.cc)
target_link_libraries(bobcat_tracked_drive_plugin ${catkin_LIBRARIES}  ${Boost_LIBRARIES} )
add_dependencies(bobcat_tracked_drive_plugin ${PROJECT_NAME}_gencfg)

add_library(bobcat_arm_plugin SHARED src/bobcat_arm_plugin.cc)
target_link_libraries(bobcat_arm_plugin ${catkin_LIBRARIES}  ${Boost_LIBRARIES} )
add_dependencies(bobcat_arm_plugin ${PROJECT_NAME}_gencfg)

add_library(sim_qinetiq_client SHARED src/sim_qinetiq_client.cc)
target_link_libraries(sim_qinetiq_client ${catkin_LIBRARIES}  ${Boost_LIBRARIES} )
add_dependencies(sim_qinetiq_client ${PROJECT_NAME}_gencfg)

add_library(noise SHARED src/noise.cc)
target_link_libraries(noise ${catkin_LIBRARIES}  ${Boost_LIBRARIES} )
add_dependencies(noise ${PROJECT_NAME}_gencfg)

add_library(chain_gazebo_plugin src/chain_plugin.cpp )
target_link_libraries(chain_gazebo_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} )


add_executable(bobcat_arm_ros_controller_interface_node src/bobcat_arm_ros_controller_interface_node.cpp)
target_link_libraries(bobcat_arm_ros_controller_interface_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES} )
