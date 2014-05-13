
cmake_minimum_required(VERSION 2.8.3)
project(wpd)
find_package(catkin REQUIRED COMPONENTS
    roscpp rospy
    robil2conf
    decision_making decision_making_parser
    geometry_msgs nav_msgs sensor_msgs tf
)
find_package(Boost REQUIRED COMPONENTS system)
catkin_package(
	CATKIN_DEPENDS
	    robil2conf
	    decision_making decision_making_parser
)
###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
## Declare a cpp library
add_library(wpd_rosComm
  src/roscomm/RosComm.cpp
)
add_library(wpd_compMain
  src/component/ComponentMain.cpp
  src/component/ComponentStates.cpp
  src/component/TwistRetranslator.cpp
  src/component/MoveBase.cpp
)
add_executable(wpd_node src/main.cpp)
target_link_libraries(wpd_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(wpd_compMain ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(wpd_node wpd_compMain wpd_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

