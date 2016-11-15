
cmake_minimum_required(VERSION 2.8.3)
project(iedsim)
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

add_library(iedsim_compMain
  src/component/ComponentMain.cpp
  src/component/ComponentStates.cpp
)

add_library(iedsim_logic
  src/component/IEDSimLogic.cpp
)

add_executable(iedsim_node src/main.cpp)
add_executable(iedsim_test src/test/iedTest.cpp)
target_link_libraries(iedsim_compMain ${catkin_LIBRARIES} ${Boost_LIBRARIES} cognitao_v2)
target_link_libraries(iedsim_node iedsim_compMain iedsim_logic ${catkin_LIBRARIES} ${Boost_LIBRARIES} cognitao_v2)
target_link_libraries(iedsim_test iedsim_compMain iedsim_logic ${catkin_LIBRARIES} ${Boost_LIBRARIES} cognitao_v2)
