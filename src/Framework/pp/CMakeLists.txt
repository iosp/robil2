
cmake_minimum_required(VERSION 2.8.3)
project(pp)
find_package(catkin REQUIRED COMPONENTS
    roscpp rospy
    robil2conf
    cognitao_io
    cognitao
    events_bus
    cognitao_event_bus_converter
    geometry_msgs nav_msgs tf
    tf_geometry
    plpcpp
    cognitao_v2
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
catkin_package(
	CATKIN_DEPENDS
	    robil2conf
	    cognitao_event_bus_converter
)
###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cognitao_v2_INCLUDE_DIRS}
)
## Declare a cpp library
#add_library(pp_rosComm
#  src/roscomm/RosComm.cpp
#)
add_library(pp_compMain
  src/component/ComponentMain.cpp
  src/component/ComponentStates.cpp
  src/component/MoveBase.cpp
)
add_executable(pp_node src/main.cpp)
<<<<<<< HEAD
#target_link_libraries(pp_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(pp_compMain ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(pp_node pp_compMain  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
#target_link_libraries(pp_node pp_compMain pp_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
=======
target_link_libraries(pp_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} cognitao_v2)
target_link_libraries(pp_compMain ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} cognitao_v2)
target_link_libraries(pp_node pp_compMain pp_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} cognitao_v2)
>>>>>>> origin/moving_to_new_cognitao


add_executable(pp_test src/tester/Tester.cpp)
target_link_libraries(pp_test ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} cognitao_v2)



