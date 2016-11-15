
cmake_minimum_required(VERSION 2.8.3)

project(llc)

find_package(catkin REQUIRED COMPONENTS
    roscpp rospy
    robil2conf
    dynamic_reconfigure
    cognitao_io
    cognitao
    events_bus
    cognitao_event_bus_converter
    cognitao_v2
)
find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(cfg/ControlParams.cfg)  

catkin_package(
	CATKIN_DEPENDS
	robil2conf
	dynamic_reconfigure
	cognitao_event_bus_converter
)

###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${cognitao_v2_INCLUDE_DIRS})
## Declare a cpp library
#add_library(llc_rosComm  src/roscomm/RosComm.cpp
#)
add_library(llc_compMain
  src/component/ComponentMain.cpp
  src/component/ComponentStates.cpp
)


add_executable(llc_node src/main.cpp)
<<<<<<< HEAD
#target_link_libraries(llc_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(llc_compMain ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(llc_node  llc_compMain  ${catkin_LIBRARIES} ${Boost_LIBRARIES})

=======
add_executable(llc_checkup src/comm_check.cpp)
target_link_libraries(llc_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES} cognitao_v2)
target_link_libraries(llc_compMain ${catkin_LIBRARIES} ${Boost_LIBRARIES} cognitao_v2)
target_link_libraries(llc_node  llc_compMain llc_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES} cognitao_v2)
target_link_libraries(llc_checkup  ${catkin_LIBRARIES} cognitao_v2)
>>>>>>> origin/moving_to_new_cognitao

#add_executable(llc_server cfg/nodes/server.cpp)
#target_link_libraries(llc_server ${catkin_LIBRARIES})
#add_dependencies(llc_server llc_generate_messages_cpp)




