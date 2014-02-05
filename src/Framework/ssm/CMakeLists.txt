
cmake_minimum_required(VERSION 2.8.3)
project(ssm)
find_package(catkin REQUIRED COMPONENTS
 roscpp rospy robil2conf
)
find_package(Boost REQUIRED COMPONENTS system)
catkin_package(
	CATKIN_DEPENDS robil2conf
)
###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
## Declare a cpp library
add_library(ssm_rosComm
  src/roscomm/RosComm.cpp
)
add_library(ssm_compMain
  src/component/ComponentMain.cpp
)
add_executable(ssm_node src/main.cpp)
target_link_libraries(ssm_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(ssm_compMain ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(ssm_node ssm_compMain ssm_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

