/*
 * HeartBeatMonitor.cpp
 *
 *  Created on: Feb 19, 2014
 *      Author: dan
 */

#include "HeartBeatMonitor.h"
#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <std_msgs/String.h>;
#include <sstream>

boost::mutex mtx;

void _on_heartbeat(HeartBeatMonitor* monitor, const std_msgs::String::ConstPtr msg ){
	boost::mutex::scoped_lock l(mtx);
	monitor->on_heartbeat(msg->data);
}
void _self_heartbeat(HeartBeatMonitor* monitor){
	using namespace boost;
	while(ros::ok()){
		double wait = (1.0/LIMIT_HZ)*1000.0*0.5;
		system_time end = get_system_time()+posix_time::milliseconds(long(wait));
		{
			boost::mutex::scoped_lock l(mtx);
			monitor->check_all();
		}
		this_thread::sleep(end);
	}
}

HeartBeatMonitor::HeartBeatMonitor() {
	ros::NodeHandle node;
	_sub_heartbeats = node.subscribe<std_msgs::String>("/heartbeat", 100, boost::bind( _on_heartbeat, this, _1));
	_pub_report = node.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 100);
}

HeartBeatMonitor::~HeartBeatMonitor() {
}

void HeartBeatMonitor::run(){
	ros::spin();
}

void HeartBeatMonitor::check_all(){
	diagnostic_msgs::DiagnosticArray msg;
	std::vector<std::string> for_remove;
	for(HeartBeatMonitor::LastTimeTracker::iterator i=_tracker.begin(); i!=_tracker.end(); i++){
		boost::system_time last_time = _tracker[i->first];
		boost::system_time now = boost::get_system_time();
		long msec = (now-last_time).total_milliseconds();
		if( msec > (1/LIMIT_HZ)*1000 ){
			msg.status.push_back( on_timeout(i->first) );
			for_remove.push_back( i->first );
		}
	}
	for(size_t i=0;i<for_remove.size();i++)_tracker.erase(for_remove[i]);
	if(msg.status.size()>0){
		sendReport(msg);
	}
}

void HeartBeatMonitor::on_heartbeat(std::string component){
	boost::system_time now = boost::get_system_time();
	if(_tracker.find(component)==_tracker.end()){
		sendReport( on_new_component(component) );
	}else{
		boost::system_time last_time = _tracker[component];
		long msec = (now-last_time).total_milliseconds();
		if( msec > (1/LIMIT_HZ)*1000 ){
			sendReport( on_returned_component(component) );
		}
	}
	_tracker[component] = now;
}

#define TAG "[SoftwareStatus:HeartBeat] "
#define COMP_NAME "Software Status"

template<class T>
diagnostic_msgs::KeyValue kv(std::string key, const T& val){
	diagnostic_msgs::KeyValue t;
	t.key = key;
	std::stringstream o; o<<val;
	t.value = o.str();
	return t;
}

diagnostic_msgs::DiagnosticStatus HeartBeatMonitor::on_timeout(std::string component){
	diagnostic_msgs::DiagnosticStatus rep;
	rep.hardware_id = "";
	rep.level = diagnostic_msgs::DiagnosticStatus::ERROR;
	rep.name = COMP_NAME;
	rep.message = TAG "Timeout of component "+component;
	rep.values.push_back(kv("component_id", component));
	rep.values.push_back(kv("status", "component timeout"));
	return rep;
}
diagnostic_msgs::DiagnosticStatus HeartBeatMonitor::on_new_component(std::string component){
	diagnostic_msgs::DiagnosticStatus rep;
	rep.hardware_id = "";
	rep.level = diagnostic_msgs::DiagnosticStatus::OK;
	rep.name = COMP_NAME;
	rep.message = TAG "New component "+component;
	rep.values.push_back(kv("component_id", component));
	rep.values.push_back(kv("status", "component started"));
	return rep;
}
diagnostic_msgs::DiagnosticStatus HeartBeatMonitor::on_returned_component(std::string component){
	diagnostic_msgs::DiagnosticStatus rep;
	rep.hardware_id = "";
	rep.level = diagnostic_msgs::DiagnosticStatus::OK;
	rep.name = COMP_NAME;
	rep.message = TAG "Component "+component+" returned";
	rep.values.push_back(kv("component_id", component));
	rep.values.push_back(kv("status", "component returned"));
	return rep;
}

void HeartBeatMonitor::sendReport(const diagnostic_msgs::DiagnosticStatus& stat){
	diagnostic_msgs::DiagnosticArray msg;
	msg.status.push_back(stat);
	sendReport(msg);
}
void HeartBeatMonitor::sendReport(const diagnostic_msgs::DiagnosticArray& msg){
	_pub_report.publish(msg);
}
int main(int ac, char** aa){
	ros::init(ac, aa, "heartbeat_monitor");
	HeartBeatMonitor monitor;
	boost::thread hb(boost::bind(_self_heartbeat,&monitor));
	monitor.run();
}
