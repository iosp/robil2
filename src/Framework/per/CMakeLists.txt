
cmake_minimum_required(VERSION 2.8.3)
project(per)
find_package(catkin REQUIRED COMPONENTS
    roscpp rospy
    cv_bridge
    robil2conf
    gazebo_ros
    decision_making decision_making_parser
)
find_package(OpenCV 2)
find_package(Boost REQUIRED COMPONENTS system)
find_package(gazebo REQUIRED)
catkin_package(
	CATKIN_DEPENDS
	    robil2conf
	    gazebo_ros
	    decision_making decision_making_parser
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
target_link_libraries(per_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(per_compMain ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(per_node per_compMain per_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${GAZEBO_libraries} 
)
link_directories(${GAZEBO_LIBRARY_DIRS})

install(
  DIRECTORY img/
  DESTINATION $ENV{HOME}/.ros/
)
