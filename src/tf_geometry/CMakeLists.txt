cmake_minimum_required(VERSION 2.8.3)
project(tf_geometry)

find_package(catkin REQUIRED COMPONENTS
  roscpp
    
  geometry_msgs nav_msgs tf
  
  eigen_conversions angles kdl_conversions tf_conversions
)

find_package(Boost REQUIRED COMPONENTS system)


################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES cryph
	CATKIN_DEPENDS roscpp geometry_msgs nav_msgs tf
)

###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

set( CRYPH_CPP
    src/cryph/AffPoint.c++
    src/cryph/AffVector.c++
    src/cryph/Basic.c++
    src/cryph/Complex.c++
    src/cryph/Formatter.c++
    src/cryph/Matrix3x3.c++
    src/cryph/Matrix4x4.c++
    src/cryph/MatrixMxN.c++
    src/cryph/MatrixNxN.c++
    src/cryph/Polynomial.c++
    src/cryph/ProjPoint.c++
    src/cryph/ViewVolume.c++
)

add_library(cryph  ${CRYPH_CPP} )
target_link_libraries(cryph  ${catkin_LIBRARIES} )

# add_library(tf_geometry  src/${PROJECT_NAME}/tf_geometry.cpp )
# add_executable(tf_test src/test.cpp)
# add_dependencies(tf_geometry_node tf_geometry_generate_messages_cpp)
# target_link_libraries(tf_test  ${catkin_LIBRARIES} cryph)
























