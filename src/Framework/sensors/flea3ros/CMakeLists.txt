cmake_minimum_required(VERSION 2.8.3)
project(flea3ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)
find_package(OpenCV 2)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES roadDetection
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/include/flycapture
)
add_executable(flea3syncros src/asyncRos.cpp)
add_executable(flea3ros src/GigEGrabEx.cpp)
add_executable(flea3config src/GigEConfig.cpp)
add_executable(saveImages src/save_images.cpp)
add_executable(flea3async src/AsyncTriggerEx.cpp)
add_executable(createMovie src/createMovie.cpp)
target_link_libraries(flea3ros ${catkin_LIBRARIES} ${OpenCV_LIBS} flycapture)
target_link_libraries(flea3async ${catkin_LIBRARIES} ${OpenCV_LIBS} flycapture)
target_link_libraries(flea3syncros ${catkin_LIBRARIES} ${OpenCV_LIBS} flycapture)
target_link_libraries(flea3config ${catkin_LIBRARIES} ${OpenCV_LIBS} flycapture)
target_link_libraries(saveImages  ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(createMovie  ${catkin_LIBRARIES} ${OpenCV_LIBS})
