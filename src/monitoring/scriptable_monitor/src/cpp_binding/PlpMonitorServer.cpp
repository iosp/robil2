/*
 * PlpMonitorServer.cpp
 *
 *  Created on: Jun 3, 2014
 *      Author: dan
 */

#include "PlpMonitorServer.h"
#include <sstream>
#include <fstream>
#include <iostream>
#include <map>
#include <std_msgs/String.h>

using namespace std;

namespace {
std::string node_ws = "/scriptable_monitor/";
std_msgs::String to_msg(const string& s){
	std_msgs::String m; m.data = s;
	return m;
}
}

PlpMonitorServer::PlpMonitorServer(ros::NodeHandle& n):node(n) {
	p_add = node.advertise<std_msgs::String>(node_ws+"add_script",10);
	p_remove = node.advertise<std_msgs::String>(node_ws+"delete_script",10);
	p_pause = node.advertise<std_msgs::String>(node_ws+"pause_module",10);
	p_resume = node.advertise<std_msgs::String>(node_ws+"resume_module",10);
}

PlpMonitorServer::~PlpMonitorServer() {
}

void PlpMonitorServer::on_event(Plp::EVENT event,const Plp* plp){
	if(plp->plp_is_repeated()){
		on_event_for_repeated(event, plp);
	}
	switch(event){
	case Plp::EVENT_MODULE_START:
		start_module(plp->get_script());
		break;
	case Plp::EVENT_MODULE_STOP:
		stop_module(plp->plp_name());
		break;
	case Plp::EVENT_GOAL_ACHIEV_START:
		//resume_module(plp->plp_name());
		break;
	case Plp::EVENT_GOAL_ACHIEV_STOP:
		//pause_module(plp->plp_name());
		break;
	}
}
void PlpMonitorServer::on_event_for_repeated(Plp::EVENT event,const Plp* plp){
	switch(event){
	case Plp::EVENT_MODULE_START:
		break;
	case Plp::EVENT_MODULE_STOP:
		stop_module(plp->plp_name());
		break;
	case Plp::EVENT_GOAL_ACHIEV_START:
		if(plp->iterations()==1){
			start_module(plp->get_script());
		}else{
			resume_module(plp->plp_name());
		}
		break;
	case Plp::EVENT_GOAL_ACHIEV_STOP:
		pause_module(plp->plp_name());
		break;
	}
}

void PlpMonitorServer::start_module(std::string script){
	p_add.publish(to_msg(script));
}
void PlpMonitorServer::stop_module(std::string module_name){
	p_remove.publish(to_msg(module_name));
}
void PlpMonitorServer::pause_module(std::string module_name){
	p_pause.publish(to_msg(module_name));
}
void PlpMonitorServer::resume_module(std::string module_name){
	p_resume.publish(to_msg(module_name));
}




