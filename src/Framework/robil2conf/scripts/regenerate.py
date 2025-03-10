#!/usr/bin/python

from datetime import datetime, date, time
import os

def gen_roscomm_header( subscribers, publishers, func_subscribers, func_publisher ):
	code = \
"""
/*
 * RosComm.h
 *
 *  Created on: """+datetime.now().strftime("%A, %d. %B %Y %I:%M%p")+"""
 *      Author: autogenerated
 */
#ifndef ROSCOMM_H_
#define ROSCOMM_H_
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>
#include <ParameterTypes.h>
#include <tf/tf.h>
#include <boost/thread.hpp>
class ComponentMain;
class RosComm {
  bool _inited;
  ComponentMain   * _comp;
  ros::NodeHandle _nh;
  ros::Publisher _pub_diagnostic;
  boost::thread_group _maintains;
""" + subscribers +"\n"+ publishers +"\n"+\
"""
  bool init(int argc,char** argv);
public:
	RosComm(ComponentMain* comp,int argc,char** argv);
	virtual ~RosComm();
""" + func_subscribers +"\n"+ func_publisher +"\n"+ \
"""
	void publishTransform(const tf::Transform& _tf, std::string srcFrame, std::string distFrame);
	tf::StampedTransform getLastTrasform(std::string srcFrame, std::string distFrame);
	void publishDiagnostic(const diagnostic_msgs::DiagnosticStatus& _report);
	void publishDiagnostic(const std_msgs::Header& header, const diagnostic_msgs::DiagnosticStatus& _report);
	void heartbeat();
};
#endif /* ROSCOMM_H_ */
"""
	return code
	
	
def get_component_header(func_subscribers, func_publisher):
	code = \
"""
/*
 * ComponentMain.h
 *
 *  Created on: """+datetime.now().strftime("%A, %d. %B %Y %I:%M%p")+"""
 *      Author: autogenerated
 */
#ifndef COMPONENTMAIN_H_
#define COMPONENTMAIN_H_
#include <std_msgs/String.h>
#include <ParameterTypes.h>
#include <tf/tf.h>
class RosComm;
class ComponentMain {
	RosComm* _roscomm;
public:
	ComponentMain(int argc,char** argv);
	virtual ~ComponentMain();
""" + func_subscribers +"\n"+ func_publisher  +\
"""
	void publishTransform(const tf::Transform& _tf, std::string srcFrame, std::string distFrame);
	tf::StampedTransform getLastTrasform(std::string srcFrame, std::string distFrame);
	void publishDiagnostic(const diagnostic_msgs::DiagnosticStatus& _report);
	void publishDiagnostic(const std_msgs::Header& header, const diagnostic_msgs::DiagnosticStatus& _report);
};
#endif /* COMPONENTMAIN_H_ */
"""
	return code

def gen_roscomm_source(comp, subs, pubs, fun_subs, fun_pubs):
	code = \
"""
/*
 * RosComm.cpp
 *
 *  Created on: """+datetime.now().strftime("%A, %d. %B %Y %I:%M%p")+"""
 *      Author: autogenerated
 */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "RosComm.h"
#include "../component/ComponentMain.h"
#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>
#include "ParameterHandler.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
RosComm::RosComm(ComponentMain* comp,int argc,char** argv)
	: _inited(init(argc, argv)), _comp(comp)
{
""" + subs +"\n"+ pubs +\
"""
	_pub_diagnostic=ros::Publisher(_nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics",100));
	_maintains.add_thread(new boost::thread(boost::bind(&RosComm::heartbeat,this)));
}
RosComm::~RosComm()
{
}
bool RosComm::init(int argc,char** argv){
	ros::init(argc,argv,\""""+comp+"""_node\");
	return true;
}

""" + fun_subs +"\n"+ fun_pubs +\
"""
void RosComm::publishTransform(const tf::Transform& _tf, std::string srcFrame, std::string distFrame){
	static tf::TransformBroadcaster br;
	br.sendTransform(tf::StampedTransform(_tf, ros::Time::now(), srcFrame, distFrame));
}
tf::StampedTransform RosComm::getLastTrasform(std::string srcFrame, std::string distFrame){
	tf::StampedTransform _tf;
	static tf::TransformListener listener;
	try {
	    listener.waitForTransform(distFrame, srcFrame, ros::Time(0), ros::Duration(10.0) );
	    listener.lookupTransform(distFrame, srcFrame, ros::Time(0), _tf);
	} catch (tf::TransformException& ex) {
	    ROS_ERROR("%s",ex.what());
	}
	return _tf;
}
void RosComm::publishDiagnostic(const diagnostic_msgs::DiagnosticStatus& _report){
	diagnostic_msgs::DiagnosticArray msg;
	msg.status.push_back(_report);
	_pub_diagnostic.publish(msg);
}
void RosComm::publishDiagnostic(const std_msgs::Header& header, const diagnostic_msgs::DiagnosticStatus& _report){
	diagnostic_msgs::DiagnosticArray msg;
	msg.header = header;
	msg.status.push_back(_report);
	_pub_diagnostic.publish(msg);
}
void RosComm::heartbeat(){
	using namespace boost::posix_time;
	ros::Publisher _pub = _nh.advertise<std_msgs::String>("/heartbeat", 10);
	double hz = HEARTBEAT_FREQUANCY;
	while(ros::ok()){
		boost::system_time stop_time = boost::get_system_time() + milliseconds((1/hz)*1000);
		std_msgs::String msg;
		msg.data = \""""+comp+"""\";
		_pub.publish(msg);
	    boost::this_thread::sleep(stop_time);
	}
}
"""
	return code


def get_component_source(fun_subs, fun_pubs):
	code = \
"""
/*
 * ComponentMain.cpp
 *
 *  Created on: """+datetime.now().strftime("%A, %d. %B %Y %I:%M%p")+"""
 *      Author: autogenerated
 */
#include "ComponentMain.h"
#include "../roscomm/RosComm.h"
ComponentMain::ComponentMain(int argc,char** argv)
{
	_roscomm = new RosComm(this,argc, argv);
}
ComponentMain::~ComponentMain() {
	if(_roscomm) delete _roscomm; _roscomm=0;
}
""" + fun_subs +"\n"+ fun_pubs +\
"""
void ComponentMain::publishTransform(const tf::Transform& _tf, std::string srcFrame, std::string distFrame){
	_roscomm->publishTransform(_tf, srcFrame, distFrame);
}
tf::StampedTransform ComponentMain::getLastTrasform(std::string srcFrame, std::string distFrame){
	return _roscomm->getLastTrasform(srcFrame, distFrame);
}
void ComponentMain::publishDiagnostic(const diagnostic_msgs::DiagnosticStatus& _report){
	_roscomm->publishDiagnostic(_report);
}
void ComponentMain::publishDiagnostic(const std_msgs::Header& header, const diagnostic_msgs::DiagnosticStatus& _report){
	_roscomm->publishDiagnostic(header, _report);
}
"""
	return code

	
def gen_roscomm_header_subscriber( comp, name ):
	return "\tros::Subscriber _sub_"+name+";"

def get_roscomm_header_subscribe_function( comp, name ):
	return "\tvoid "+name+"Callback(const config::"+comp+"::sub::"+name+"::ConstPtr &msg);"


def get_component_header_subscribe_function( comp, name ):
	return "\tvoid handle"+name+"(const config::"+comp+"::sub::"+name+"& msg);"


def gen_roscomm_source_subscriber( comp, name ):
	return "\t_sub_"+name+"=ros::Subscriber(_nh.subscribe(fetchParam(&_nh,\""+comp+"\",\""+name+"\",\"sub\"), 10, &RosComm::"+name+"Callback,this));"

def get_roscomm_source_subscribe_function( comp, name ):
	code=\
	"""
void RosComm::"""+name+"""Callback(const config::"""+comp+"""::sub::"""+name+"""::ConstPtr &msg)
{
	_comp->handle"""+name+"""(*msg);
}
	"""
	return code

def get_component_source_subscribe_fun( comp, name ):
	code=\
	"""
void ComponentMain::handle"""+name+"""(const config::"""+comp+"""::sub::"""+name+"""& msg)
{
	std::cout<< \""""+comp+""" say:" << msg << std::endl;
}
	"""
	return code

def gen_roscomm_header_publisher( comp, name ):
	return "\tros::Publisher  _pub_"+name+";"

def get_roscomm_header_publish_function( comp, name ):
	return "\tvoid publish"+name+"( config::"+comp+"::pub::"+name+" &msg);"


def get_component_header_publish_function( comp, name ):
	return "\tvoid publish"+name+"(config::"+comp+"::pub::"+name+"& msg);"


def gen_roscomm_source_publisher( comp, name ):
	return "\t_pub_"+name+"=ros::Publisher(_nh.advertise<config::"+comp+"::pub::"+name+">(fetchParam(&_nh,\""+comp+"\",\""+name+"\",\"pub\"),10));"

def get_roscomm_source_publish_function( comp, name ):
	code =\
	"""
void RosComm::publish"""+name+"""( config::"""+comp+"""::pub::"""+name+""" &msg)
{
	_pub_"""+name+""".publish(msg);
}
	"""
	return code

def get_component_source_publish_function( comp, name ):
	code =\
	"""
void ComponentMain::publish"""+name+"""(config::"""+comp+"""::pub::"""+name+"""& msg)
{
	_roscomm->publish"""+name+"""(msg);
}
	"""
	return code


class CodeGen:
	def __init__(self, comp):
		self.component = comp
		self.all_roscomm_header_subscribers = []
		self.all_roscomm_header_publishers = []
		self.all_roscomm_header_subscribe_functions = []
		self.all_roscomm_header_publish_functions = []
		self.all_component_header_subscribe_functions = []
		self.all_component_header_publish_function = []
		self.all_roscomm_source_subscribers = []
		self.all_roscomm_source_publishers = []
		self.all_roscomm_source_subscribe_functions = []
		self.all_roscomm_source_publish_functions = []
		self.all_component_source_subscribe_functions = []
		self.all_component_source_publish_functions = []
	def addSub( self, name ):
		self.all_roscomm_header_subscribers.append( gen_roscomm_header_subscriber(self.component,name) )
		self.all_roscomm_header_subscribe_functions.append( get_roscomm_header_subscribe_function(self.component,name) )
		self.all_component_header_subscribe_functions.append( get_component_header_subscribe_function(self.component,name) )
		self.all_roscomm_source_subscribers.append( gen_roscomm_source_subscriber(self.component,name) )
		self.all_roscomm_source_subscribe_functions.append( get_roscomm_source_subscribe_function(self.component,name) )
		self.all_component_source_subscribe_functions.append( get_component_source_subscribe_fun(self.component,name) )
	def addPub( self, name ):
		self.all_roscomm_header_publishers.append( gen_roscomm_header_publisher(self.component,name) )
		self.all_roscomm_header_publish_functions.append( get_roscomm_header_publish_function(self.component,name) )
		self.all_component_header_publish_function.append( get_component_header_publish_function(self.component,name) )
		self.all_roscomm_source_publishers.append( gen_roscomm_source_publisher(self.component,name) )
		self.all_roscomm_source_publish_functions.append( get_roscomm_source_publish_function(self.component,name) )
		self.all_component_source_publish_functions.append( get_component_source_publish_function(self.component,name) )

	def get_roscomm_header(self):
		return gen_roscomm_header(
			"\n".join(self.all_roscomm_header_subscribers), "\n".join(self.all_roscomm_header_publishers),
			"\n".join(self.all_roscomm_header_subscribe_functions), "\n".join(self.all_roscomm_header_publish_functions)
			)
	def get_component_header(self):
		return get_component_header("\n".join(self.all_component_header_subscribe_functions),"\n".join(self.all_component_header_publish_function))

	def get_roscomm_source(self):
		return gen_roscomm_source( self.component,
			"\n".join(self.all_roscomm_source_subscribers),"\n".join(self.all_roscomm_source_publishers),
			"\n".join(self.all_roscomm_source_subscribe_functions),"\n".join(self.all_roscomm_source_publish_functions)
			)
	def get_component_source(self):
		return get_component_source("\n".join(self.all_component_source_subscribe_functions),"\n".join(self.all_component_source_publish_functions))

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
	
#==================== MAIN ========================================	

if not is_project():
	print "current folder does not a component folder"
	import sys
	sys.exit(1)

comp = component_name() 
print "Component name : "+comp

code = CodeGen(comp)

conf_path = path_to_configuration()
conf = open(conf_path ,'r').readlines()
comp_conf = [ line[:-len(line)+line.index(':')].strip()[len(comp)+1:] for line in conf if line.startswith(comp) ]
comp_sub_conf = [ line[4:] for line in comp_conf if line.startswith('sub') ]
comp_pub_conf = [ line[4:] for line in comp_conf if line.startswith('pub') ]

print "output topics:"
for e in comp_sub_conf:
	print "\t",e
	code.addSub(e)
print "input topics:"
for e in comp_pub_conf:
	print "\t",e
	code.addPub(e)
	
path_to_component_header = 'src/component/ComponentMain.h'
path_to_component_source = 'src/component/ComponentMain.cpp'
path_to_roscomm_header = 'src/roscomm/RosComm.h'
path_to_roscomm_source = 'src/roscomm/RosComm.cpp'

print "Generate headers ... ",
open( path_to_roscomm_header, 'w' ).write( code.get_roscomm_header() )
open( path_to_component_header, 'w').write( code.get_component_header() )
print "Done"
print "Generate sources ... ",
open( path_to_roscomm_source, 'w' ).write( code.get_roscomm_source() )
open( path_to_component_source, 'w' ).write( code.get_component_source() )
print "Done"










