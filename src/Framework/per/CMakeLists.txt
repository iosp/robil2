
cmake_minimum_required(VERSION 2.8.3)
project(per)
find_package(catkin REQUIRED COMPONENTS
    roscpp rospy
    cv_bridge
    robil2conf
    gazebo_ros
    decision_making decision_making_parser
message_generation
image_transport
dynamic_reconfigure
)
find_package(OpenCV 2)
find_package(Boost REQUIRED COMPONENTS system)
find_package(gazebo REQUIRED)


add_message_files(
  FILES
  lane.msg
  roadLanes.msg
)
generate_dynamic_reconfigure_options(
  cfg/config.cfg
)
generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
	CATKIN_DEPENDS
	    robil2conf
	    gazebo_ros
	    decision_making decision_making_parser
	    message_runtime
)
###########
## Build ##
###########
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)
## Declare a cpp library
add_library(per_rosComm
  src/roscomm/RosComm.cpp
)

add_library(per_compMain
  src/component/ComponentMain.cpp
  src/component/ComponentStates.cpp
  src/component/heightmap.cpp 
  src/component/heightmap_projection.cpp 
  src/component/mapper/mapper.cpp 
  src/component/helpermath.cpp
)
link_directories(${GAZEBO_LIBRARY_DIRS})
add_executable(per_node src/main.cpp)
add_dependencies(per_node ${PROJECT_NAME}_gencfg)
target_link_libraries(per_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(per_compMain ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(per_node per_compMain per_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${GAZEBO_libraries} 
)
link_directories(${GAZEBO_LIBRARY_DIRS})

# add_library(dispImage
#   src/road_detection/displayImage/DisplayImage.cpp
# )
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(road_detection src/road_detection/road_detection.cpp)
target_link_libraries(road_detection ${catkin_LIBRARIES} ${OpenCV_LIBS})
# target_link_libraries(dispImage ${catkin_LIBRARIES})
# target_link_libraries(road_detection dispImage ${catkin_LIBRARIES} ${OpenCV_LIBS})


install(
  DIRECTORY img/
  DESTINATION $ENV{HOME}/.ros/
)
