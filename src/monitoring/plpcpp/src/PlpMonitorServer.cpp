/*
 * PlpMonitorServer.cpp
 *
 *  Created on: Jun 3, 2014
 *      Author: dan
 */

#include <plpcpp/PlpMonitorServer.h>
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

namespace plp{

PlpMonitorServer::PlpMonitorServer(ros::NodeHandle& n):_simulate(0), node(n) {
	p_add = node.advertise<std_msgs::String>(node_ws+"add_script",10);
	p_remove = node.advertise<std_msgs::String>(node_ws+"delete_script",10);
	p_pause = node.advertise<std_msgs::String>(node_ws+"pause_module",10);
	p_resume = node.advertise<std_msgs::String>(node_ws+"resume_module",10);
}

PlpMonitorServer::PlpMonitorServer(ros::NodeHandle& n, int _sim):_simulate(_sim),node(n) {
	if(_simulate>1) return;
	p_add = node.advertise<std_msgs::String>(node_ws+"add_script",10);
	p_remove = node.advertise<std_msgs::String>(node_ws+"delete_script",10);
	p_pause = node.advertise<std_msgs::String>(node_ws+"pause_module",10);
	p_resume = node.advertise<std_msgs::String>(node_ws+"resume_module",10);
}

PlpMonitorServer::~PlpMonitorServer() {
}

void PlpMonitorServer::on_event(Module::EVENT event,const Module* plp){
	if(plp->plp_is_repeated()){
		on_event_for_repeated(event, plp);
		return;
	}
	std::cout<<"[d] on_event_for_unrepeated"<<std::endl;
	switch(event){
	case Module::EVENT_MODULE_START:
		start_module(plp->get_script());
		break;
	case Module::EVENT_MODULE_STOP:
		stop_module(plp->plp_name());
		break;
	case Module::EVENT_GOAL_ACHIEV_START:
		resume_module(plp->plp_name());
		break;
	case Module::EVENT_GOAL_ACHIEV_STOP:
		pause_module(plp->plp_name());
		break;
	}
}
void PlpMonitorServer::on_event_for_repeated(Module::EVENT event,const Module* plp){
	std::cout<<"[d] on_event_for_repeated"<<std::endl;
	switch(event){
	case Module::EVENT_MODULE_START:
		//start_module(plp->get_script());
		break;
	case Module::EVENT_MODULE_STOP:
		stop_module(plp->plp_name());
		break;
	case Module::EVENT_GOAL_ACHIEV_START:
		if(plp->iterations()==1){
			start_module(plp->get_script());
		}else{
			resume_module(plp->plp_name());
		}
		break;
	case Module::EVENT_GOAL_ACHIEV_STOP:
		pause_module(plp->plp_name());
		break;
	}
}

void PlpMonitorServer::start_module(const std::string& script){
	if(_simulate){size_t i=script.find("PLP:");std::cout<<"start_module "<<replace_all_copy(script.substr(i+5,script.find('\n',i)-i-5)," ","_")<<endl; if(_simulate>1) return;}
	p_add.publish(to_msg(script));
}
void PlpMonitorServer::stop_module(const std::string& module_name){
	if(_simulate){cout<<"stop_module "<< module_name<<endl;if(_simulate>1)return;}
	p_remove.publish(to_msg(module_name));
}
void PlpMonitorServer::pause_module(const std::string& module_name){
	if(_simulate){cout<<"pause_module "<< module_name<<endl;if(_simulate>1)return;}
	p_pause.publish(to_msg(module_name));
}
void PlpMonitorServer::resume_module(const std::string& module_name){
	if(_simulate){cout<<"resume_module "<< module_name<<endl;if(_simulate>1)return;}
	p_resume.publish(to_msg(module_name));
}

}


