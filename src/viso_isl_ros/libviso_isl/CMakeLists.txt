cmake_minimum_required(VERSION 2.8.3)
project(libviso_isl)
find_package(catkin REQUIRED )
set(CMAKE_CXX_FLAGS "-std=c++0x")
ADD_DEFINITIONS(-DBOOST_ALL_DYN_LINK)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(Eigen3 3.1.2 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
set(Boost_USE_STATIC_LIBS OFF)  # I've already tried ON
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS system filesystem thread REQUIRED)

include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

#set(CMAKE_CXX_FLAGS "-msse3")
add_definitions(-msse3)

catkin_package(
  INCLUDE_DIRS libviso_isl/src
  LIBRARIES viso_isl
)

include_directories(libviso_isl/src
  ${catkin_INCLUDE_DIRS}
)
add_library(viso_isl libviso_isl/src/viso.cpp libviso_isl/src/mvg.cpp libviso_isl/src/estimation.cpp libviso_isl/src/misc.cpp libviso_isl/src/nms.cpp)

# add_library(viso_isl
#   libviso_isl/src/filter.cpp
#   libviso_isl/src/matcher.cpp
#   libviso_isl/src/matrix.cpp
#   libviso_isl/src/reconstruction.cpp
#   libviso_isl/src/triangle.cpp
#   libviso_isl/src/viso.cpp
#   libviso_isl/src/viso_mono.cpp
#   libviso_isl/src/viso_stereo.cpp)

install(TARGETS viso_isl
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ${PROJECT_NAME}/src
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".cpp" EXCLUDE
)

