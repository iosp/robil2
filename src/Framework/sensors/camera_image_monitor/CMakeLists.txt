cmake_minimum_required(VERSION 2.8.3)
project(camera_image_monitor)

find_package(catkin REQUIRED COMPONENTS
 roscpp rospy robil2conf cv_bridge
)
if(("$ENV{ROS_DISTRO}" STREQUAL "kinetic") OR ("$ENV{ROS_DISTRO}" STRGREATER "kinetic"))
find_package(OpenCV REQUIRED)
else()
find_package(OpenCV 2)
endif()


find_package(Boost REQUIRED COMPONENTS system)
catkin_package(
	CATKIN_DEPENDS robil2conf
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
###########
## Build ##
###########





## Declare a cpp executable
 add_executable(camera_image_monitor_node src/main.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(camera_image_monitor_node camera_image_monitor_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(camera_image_monitor_node
   ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES}
 )


