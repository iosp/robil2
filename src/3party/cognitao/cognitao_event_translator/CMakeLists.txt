cmake_minimum_required(VERSION 2.8.3)
project(cognitao_event_translator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  events_bus
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(cognitao_event_translator
  src/cognitao_event_translator.cpp
)

target_link_libraries(cognitao_event_translator ${catkin_LIBRARIES} events_bus)