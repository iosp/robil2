cmake_minimum_required(VERSION 2.8.3)
project(robil2rqt)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS nodes/velocityPublisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
install(PROGRAMS nodes/pubWPDCommand.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
