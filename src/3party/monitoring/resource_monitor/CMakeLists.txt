cmake_minimum_required(VERSION 2.8.3)
project(resource_monitor)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  roscpp
  rospy
)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(PROGRAMS
  src/resource_monitor/resource_monitor_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )