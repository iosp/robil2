
cmake_minimum_required(VERSION 2.8.3)
project(loc)
find_package(catkin REQUIRED COMPONENTS
 roscpp rospy robil2conf cv_bridge
)
find_package(OpenCV 2)
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
  ${OpenCV_INCLUDE_DIRS}
)
## Declare a cpp library
add_library(loc_rosComm
  src/roscomm/RosComm.cpp
)
add_library(loc_compMain
  src/component/ComponentMain.cpp
  src/component/ekf_class.cpp
  src/component/noiseless_estimator.cpp
  src/userHelperFunction/example.cpp
)
add_executable(loc_node src/main.cpp)
add_executable(user_function_example src/userHelperFunction/example.cpp)
target_link_libraries(loc_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(loc_compMain ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(loc_node loc_compMain loc_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(user_function_example loc_compMain loc_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

