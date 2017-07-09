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
    roscpp rospy
    robil2conf
    decision_making decision_making_parser
)
find_package(Boost REQUIRED COMPONENTS system)
catkin_package(
	CATKIN_DEPENDS
	    robil2conf
	    decision_making decision_making_parser
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
  src/component/ComponentStates.cpp
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
	<build_depend>decision_making</build_depend>
	<build_depend>decision_making_parser</build_depend>
	<run_depend>roscpp</run_depend>
	<run_depend>robil2conf</run_depend>
	<run_depend>decision_making</run_depend>
	<run_depend>decision_making_parser</run_depend>
  <export>
  </export>
</package>
'''
		return code
	def text_main_cpp(self):
		code = '''\
#include <ros/ros.h>
#include "component/ComponentMain.h"
#include "component/ComponentStates.h"
#include <ros/spinner.h>
#include <boost/thread/thread.hpp>
int main(int argc,char** argv)
{
    ComponentMain comp(argc,argv);
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    //ros::waitForShutdown();
    runComponent(argc,argv, comp);
    return 0;
}
'''
		return code
	def text_component_h(self):
		code = '''\
#ifndef COMPONENTSTATES_H_
#define COMPONENTSTATES_H_
#include "ComponentMain.h"
void runComponent(int argc, char** argv, ComponentMain& component);
void runComponent(int argc, char** argv, ComponentMain* component){runComponent(argc, argv, *component);}
#endif /* COMPONENTSTATES_H_ */
'''
		return code

	def text_component_cpp(self):
		code = '''\
#include <iostream>
#include <ros/ros.h>
#include <decision_making/SynchCout.h>
#include <decision_making/BT.h>
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>
using namespace std;
using namespace decision_making;
#include "ComponentStates.h"

class Params: public CallContextParameters{
public:
	ComponentMain* comp;
	Params(ComponentMain* comp):comp(comp){}
	std::string str()const{return "";}
};

//// ============== WRITE FSM HERE ========================= /////

TaskResult state_OFF(string id, const CallContext& context, EventQueue& events){
	PAUSE(10000);
	return TaskResult::SUCCESS();
}
TaskResult state_INIT(string id, const CallContext& context, EventQueue& events){
	PAUSE(10000);
	return TaskResult::SUCCESS();
}
TaskResult state_READY(string id, const CallContext& context, EventQueue& events){
	PAUSE(10000);
	return TaskResult::SUCCESS();
}
TaskResult state_STANDBY(string id, const CallContext& context, EventQueue& events){
	PAUSE(10000);
	return TaskResult::SUCCESS();
}

void runComponent(int argc, char** argv, ComponentMain& component){

	ros_decision_making_init(argc, argv);
	RosEventQueue events;
	CallContext context;
	context.createParameters(new Params(&component));
	//events.async_spin();
	LocalTasks::registration("OFF",state_OFF);
	LocalTasks::registration("INIT",state_INIT);
	LocalTasks::registration("READY",state_READY);
	LocalTasks::registration("STANDBY",state_STANDBY);

	ROS_INFO("Starting '''+self.name+'''...");
	Fsm'''+self.name+'''(&context, &events);

}
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
main_cpp = comp.text_main_cpp()

comp_h = comp.text_component_h()
comp_cpp = comp.text_component_cpp()

print "Generate CMakeLists.txt, package.xml and main.cpp etc ... ",

open( "CMakeLists.txt", 'w' ).write( cmakelist )
open( "package.xml", 'w' ).write( package )
open( "src/main.cpp", 'w' ).write( main_cpp )
open( "src/main.cpp", 'w' ).write( main_cpp )
open( "src/component/ComponentStates.h", 'w' ).write( comp_h )
open( "src/component/ComponentStates.cpp", 'w' ).write( comp_cpp )

print "Done"




















