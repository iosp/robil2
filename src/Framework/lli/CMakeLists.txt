
cmake_minimum_required(VERSION 2.8.3)
project(lli)
find_package(catkin REQUIRED COMPONENTS
    roscpp rospy
    robil2conf
    cognitao_io
    cognitao
    events_bus
    cognitao_event_bus_converter
    cognitao_v2
    
)
find_package(Boost REQUIRED COMPONENTS system)
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
  ${cognitao_v2_INCLUDE_DIRS}
)
## Declare a cpp library
#add_library(lli_rosComm
#  src/roscomm/RosComm.cpp
#)
add_library(LLICtrl 
  src/QinetiQ_IO/LLICtrl.cpp
  src/QinetiQ_IO/ShareUtil.cpp 
)
add_library(lli_compMain
  src/component/ComponentMain.cpp
  src/component/ComponentStates.cpp
 ## src/component/lliCtrlManager.cpp
)

add_executable(lli_node src/main.cpp)
<<<<<<< HEAD
#target_link_libraries(lli_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(lli_compMain ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(lli_node lli_compMain  LLICtrl ${catkin_LIBRARIES} ${Boost_LIBRARIES}
=======
target_link_libraries(lli_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES} cognitao_v2)
target_link_libraries(lli_compMain ${catkin_LIBRARIES} ${Boost_LIBRARIES} cognitao_v2)
target_link_libraries(lli_node lli_compMain lli_rosComm LLICtrl ${catkin_LIBRARIES} ${Boost_LIBRARIES} cognitao_v2
>>>>>>> origin/moving_to_new_cognitao
)


