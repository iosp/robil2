
cmake_minimum_required(VERSION 2.8.3)
project(loc)
find_package(catkin REQUIRED COMPONENTS
 roscpp rospy robil2conf cv_bridge dynamic_reconfigure
)
if(("$ENV{ROS_DISTRO}" STREQUAL "kinetic") OR ("$ENV{ROS_DISTRO}" STRGREATER "kinetic"))
find_package(OpenCV REQUIRED)
else()
find_package(OpenCV 2)
endif()

find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(
  cfg/config.cfg
)
catkin_package(
	CATKIN_DEPENDS robil2conf
)
###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)
## Declare a cpp library
#add_library(loc_rosComm
#  src/roscomm/RosComm.cpp
#)
add_library(loc_compMain
#  src/component/ComponentMain.cpp
#  src/component/ekf_class.cpp
#  src/component/noiseless_estimator.cpp
#  src/userHelperFunction/example.cpp
src/localization.cpp
)
add_executable(loc_node src/main.cpp)
add_executable(user_function_example src/userHelperFunction/example.cpp)
add_dependencies(loc_node ${PROJECT_NAME}_gencfg)
#target_link_libraries(loc_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(loc_compMain ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(loc_node loc_compMain  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(user_function_example loc_compMain  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})
#target_link_libraries(loc_node loc_compMain loc_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})
#target_link_libraries(user_function_example loc_compMain loc_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

