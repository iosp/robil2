# project
cmake_minimum_required (VERSION 2.6)
project (libviso_isl)

# directories
set (LIBVISO_ISL_SRC_DIR src)

# include directory
include_directories("${LIBVISO_ISL_SRC_DIR}")

# use sse3 instruction set
SET(CMAKE_CXX_FLAGS "-msse3")

# sources
FILE(GLOB LIBVISO_ISL_SRC_FILES "src/*.cpp")

# make release version
set(CMAKE_BUILD_TYPE Release)

# demo program
#add_executable(viso_isl ${LIBVISO_ISL_SRC_FILES})
#target_link_libraries (viso_isl png)

