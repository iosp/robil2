cmake_minimum_required(VERSION 2.8.3)
project(navex)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  dynamic_reconfigure
  pcl_ros
)

find_package(OpenCV REQUIRED COMPONENTS
  core
  imgproc
  highgui
)

generate_dynamic_reconfigure_options(
  cfg/CostMap.cfg
  cfg/CostMapPCData.cfg
)

catkin_package(  
  INCLUDE_DIRS include
  LIBRARIES navex
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(navex 
  src/CostMapCell.cpp
  src/CostMap.cpp
  src/CostMapDataContainer.cpp
  src/LaserScanDataSource.cpp
  src/RosTopicMapDataSource.cpp
  src/SimpleTrajectoryMatcher.cpp
  src/GoalTrajectoryMatcher.cpp
  src/Trajectory.cpp
  src/TrajectoryMatch.cpp
  src/TrajectorySimulator.cpp
  src/path_search/AStar.cpp
)

target_link_libraries(navex 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

##########
## Test ##
##########

if(CATKIN_ENABLE_TESTING)

    add_executable(ros_topic_map_source_test_node test/nodes/ros_topic_map_source_test_node.cpp)
    target_link_libraries(ros_topic_map_source_test_node navex)
    
    add_executable(local_costmap_test_node test/nodes/local_costmap_test_node.cpp)
    target_link_libraries(local_costmap_test_node navex)
    
    add_executable(a_star_test_node test/nodes/a_star_test_node.cpp)
    target_link_libraries(a_star_test_node navex)
    
    add_executable(cloud_data_source_test_node test/nodes/cloud_data_source_test_node.cpp)
    target_link_libraries(cloud_data_source_test_node navex)

    catkin_add_gtest(utest test/utest.cpp)
    target_link_libraries(utest 
    	${catkin_LIBRARIES}
    	${OpenCV_LIBRARIES}
    )
    
endif()

#############
## Install ##
#############

install(TARGETS navex
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
