set(CMAKE_CXX_FLAGS "-std=c++0x")

ADD_DEFINITIONS(-DBOOST_ALL_DYN_LINK)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 3.1.2 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(Boost_USE_STATIC_LIBS OFF)  # I've already tried ON
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS log log_setup system filesystem thread REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

add_library(viso viso.cpp mvg.cpp estimation.cpp misc.cpp nms.cpp)

add_executable(kitti kitti.cpp)
target_link_libraries(kitti viso ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(isl isl.cpp)
target_link_libraries(isl viso ${OpenCV_LIBS} ${Boost_LIBRARIES})
