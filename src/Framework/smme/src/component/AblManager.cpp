/*
 * AblManager.cpp
 *
 *  Created on: Mar 20, 2014
 *      Author: dan
 */

#include "AblManager.h"
#include <decision_making/EventSystem.h>
using namespace decision_making;

AblManager::AblManager(ComponentMain* comp)
:
	component(comp)

{
#define ADD_TRIGGER(N,C) \
	all_triggers[N]=(Trigger(N,C));\
	all_compliments[C]=N;\
	all_triggers_names.insert(N);all_triggers_names.insert(C);

	ADD_TRIGGER("/NoPathFound","/PathFound");
	ADD_TRIGGER("/CommFail","/CommOK");
	ADD_TRIGGER("/CommFail","/CommOK");
	ADD_TRIGGER("/ObstacleDetected","/AllClear");
	ADD_TRIGGER("/RoadDetected","/OpenSpace");
	ADD_TRIGGER("/Turn-over","/StablePosition");
	ADD_TRIGGER("/Collision","/NoCollisions");
	ADD_TRIGGER("/NoGPS","/GPSOK");
	ADD_TRIGGER("/AssistanceRequired","");

#undef ADD_TRIGGER
}

AblManager::~AblManager() {
}


bool AblManager::is_abl_event(std::string event_name)const{
	return all_triggers_names.find(event_name)!=all_triggers_names.end();
}
bool AblManager::is_trigger(std::string event_name)const{
	return all_triggers.find(event_name)!=all_triggers.end();
}

bool AblManager::on_trigger(std::string event_name){
	std::string def="_UNKNOWN_";
	std::string trigger_policy;
	std::string trigger_name = "/ABL/Events/"+event_name;
	ros::param::param(trigger_name,trigger_policy,def);
	if(trigger_policy==def) return false;
	activate(all_triggers[trigger_name],trigger_policy);
	return true;
}

bool AblManager::on_compliment(std::string event_name){
	deactivate(event_name);
	return true;
}
bool AblManager::on_event(std::string event_name){
	if(is_trigger(event_name)) return on_trigger(event_name);
	return on_compliment(event_name);
}
void AblManager::listen(decision_making::EventQueue* events){
	while(ros::ok() and events->isTerminated()==false){
		Event event = events->waitEvent();
		if(not event) continue;
		std::string event_name = event.name();
		if(is_abl_event(event_name))
			on_event(event_name);
	}
}

void AblManager::activate(const Trigger& trigger,const std::string& polyci){
	BOOST_FOREACH(Activated act, activated){
		if(act.polyci == polyci) return;
	}
	Activated act={trigger,polyci};
	activated.push_back(act);
	on_activation(act);
}

void AblManager::deactivate(const std::string& compliment){
	BOOST_FOREACH(Activated act, activated){
		if(act.trigger.compliment == compliment){
			on_deactivation(act);
		}
	}
}

void AblManager::on_activation(const Activated& act){

}
void AblManager::on_deactivation(const Activated& act){

}





