cmake_minimum_required(VERSION 2.8.3)
project(cognitao_v2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cognitao
  cognitao_io
  events_bus
  cognitao_event_bus_converter
  message_generation
)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cognitao_v2
  CATKIN_DEPENDS roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

add_library(cognitao_v2
  src/cognitao_v2.cpp
)

add_dependencies(cognitao_v2  cognitao_generate_messages_cpp)
target_link_libraries(cognitao_v2 ${catkin_LIBRARIES} ${Boost_LIBRARIES} cognitao events_bus)