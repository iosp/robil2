
cmake_minimum_required(VERSION 2.8.3)

project(llc)

find_package(catkin REQUIRED COMPONENTS
    roscpp rospy
    robil2conf
    decision_making decision_making_parser
    dynamic_reconfigure
)
find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(cfg/ControlParams.cfg)  

catkin_package(
	CATKIN_DEPENDS
	robil2conf
	decision_making
	decision_making_parser
	dynamic_reconfigure
)

###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
## Declare a cpp library
add_library(llc_rosComm  src/roscomm/RosComm.cpp
)
add_library(llc_compMain
  src/component/ComponentMain.cpp
  src/component/ComponentStates.cpp
)


add_executable(llc_node src/main.cpp)
target_link_libraries(llc_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(llc_compMain ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(llc_node llc_compMain llc_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(llc_server cfg/nodes/server.cpp)
#target_link_libraries(llc_server ${catkin_LIBRARIES})
#add_dependencies(llc_server llc_generate_messages_cpp)




