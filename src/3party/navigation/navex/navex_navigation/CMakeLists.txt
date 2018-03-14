cmake_minimum_required(VERSION 2.8.3)
project(navex_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  navex
  dynamic_reconfigure
  move_base_msgs
  pcl_ros
)

find_package(OpenCV REQUIRED COMPONENTS
  imgproc
  highgui
)

generate_dynamic_reconfigure_options(
  cfg/NavigationParameters.cfg
  cfg/NavexLocalPlanner.cfg
  cfg/GoalTrajectoryMatcher.cfg
)



catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
) 

add_executable(navex_navigation_node 
  src/navex_navigation_node.cpp
  src/NavexNavigation.cpp
  src/NavexLocalPlanner.cpp
  src/NavexGlobalPlanner.cpp
  src/NavexPathFinder.cpp
  src/MoveBackwardRecovery.cpp
  src/NavigateBackwardRecovery.cpp
  src/LocalGoalFinder.cpp
  src/FuzzyTrajectoryMatcher.cpp
  src/FuzzyLocalPlanner.cpp
  src/FuzzyGlobalPlanner.cpp
  
  src/path_manager/PathManager.cpp
  
  src/path_damping/PathPlanAlphaBetaFilter.cpp
  src/path_damping/PathDamping.cpp
)

add_dependencies(navex_navigation_node ${PROJECT_NAME}_gencfg)

target_link_libraries(navex_navigation_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#############
## Install ##
#############

install(TARGETS navex_navigation_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

INSTALL(DIRECTORY
	launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
