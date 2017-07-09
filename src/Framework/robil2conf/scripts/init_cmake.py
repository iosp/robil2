#!/usr/bin/python

from datetime import datetime, date, time
import os


def path_to_configuration():
	path_to_conf = os.popen("pwd").read()
	path_to_conf = path_to_conf[:-len(path_to_conf)+path_to_conf.index("robil2/src")]
	path_to_conf = path_to_conf + "robil2/src/Framework/robil2conf/configuration.yaml"
	return path_to_conf

def component_name():
	path = os.popen("pwd").read()
	i = path.index("robil2/src/Framework/")+len("robil2/src/Framework/")
	#j = len(path)-path.index("/",i+1)
	path = path[i:]
	return path.upper().strip()

def is_project():
	r = os.popen('find . -name RosComm.h').read().strip()
	return r == "./src/roscomm/RosComm.h"
	
class Component:
	def __init__(self, name):
		self.name = name
	def text_CMakeList(self):
		code='''
cmake_minimum_required(VERSION 2.8.3)
project('''+self.name+''')
find_package(catkin REQUIRED COMPONENTS
 roscpp rospy robil2conf
)
find_package(Boost REQUIRED COMPONENTS system)
catkin_package(
	CATKIN_DEPENDS robil2conf
)
###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
## Declare a cpp library
add_library('''+self.name+'''_rosComm
  src/roscomm/RosComm.cpp
)
add_library('''+self.name+'''_compMain
  src/component/ComponentMain.cpp
)
add_executable('''+self.name+'''_node src/main.cpp)
target_link_libraries('''+self.name+'''_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries('''+self.name+'''_compMain ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries('''+self.name+'''_node '''+self.name+'''_compMain '''+self.name+'''_rosComm ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

'''
		return code
	def text_package(self):
		code = '''\
<?xml version="1.0"?>
<package>
  <name>'''+self.name+'''</name>
  <version>0.0.0</version>
  <description>The '''+self.name+''' package</description>
  <maintainer email="yuval@todo.todo">yuval</maintainer>
  <license>TODO</license>
  <buildtool_depend>catkin</buildtool_depend>
    <build_depend>roscpp</build_depend>
    <build_depend>robil2conf</build_depend>
  	<run_depend>roscpp</run_depend>
  	<run_depend>robil2conf</run_depend>
  <export>
  </export>
</package>
'''
		return code

#==================== MAIN ========================================	

if not is_project():
	print "current folder does not a component folder"
	import sys
	sys.exit(1)

comp_name = component_name().lower()
print "Component name : "+comp_name

comp = Component(comp_name)

cmakelist = comp.text_CMakeList()
package = comp.text_package()

print "Generate CMakeLists.txt and package.xml ... ",

open( "CMakeLists.txt", 'w' ).write( cmakelist )
open( "package.xml", 'w' ).write( package )

print "Done"




















