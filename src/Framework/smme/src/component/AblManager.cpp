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
#define USE_DEFAULT 1
#if(USE_DEFAULT==1)
	std::string bydefault="1";
#else
	std::string bydefault="_UNKNOWN_";
#endif
	std::string trigger_policy;
	std::string trigger_name = "/ABL/Events"+event_name;
	ros::param::param(trigger_name,trigger_policy,bydefault);
#if(USE_DEFAULT==0)
	if(trigger_policy==bydefault) return false;
#endif
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
		if(act.policy == polyci) return;
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
/**
Comm. Fail
	SSM External communication failure for more than 10 seconds.
		1. Continue mission autonomously
		2. Pause mission until recovery
		3. Drive back to last comm. point and wait for commands (reverse route)
Obstacle detected
	WPD/PP? Obstacle detected in path
		1. Avoid obstacle
		2. Start predefined W.S to remove obstacle
		3. Pause and wait for operator
Road detected
	WPD/PP(?) Road shoulders detected in path
		1.Correct path and drive along it
		2.Ignore detection
No autonomous solution/ road blocked
	WPD/PP(?) When the UGV cannot calculate route due to lack of information, data inconsistency or path blocked. 5m ahead
		1.(optional) stop and reset map/go back one step/recalculate mission
		2. Abort mission and go back (reverse route)
		3. Pause and wait for operator
Turn-over
	SSM Stopping at critical pitch/roll values. 5 degrees under max allowed
		1.Recalculate and continue mission
		2. Pause and wait for operator
Collision
	LLC Vehicle not moving despite receiving speed from WPD
		1.Recalculate and continue mission
		2. Pause and wait for operator
No GPS signal
	Localization No GPS signal for more than 10 seconds
		1. Navigate without GPS
		2. Navigate without GPS for X meters
		3. Pause and wait for operator
**/
#define switch_str(NAME) const string& _sw_=NAME; bool _t_=true; if(false)
#define case_str(VAL) }else if(VAL==_sw_){
static const string POLICY_AUTONOMY=std::string("1");
static const string POLICY_MEDIUM=std::string("2");
static const string POLICY_TELEOPERATION=std::string("3");

void AblManager::on_activation(const Activated& act){
	decision_making::EventQueue* events = this->component->events();
	if(!events) return;
	switch_str(act.trigger.name){
		case_str( "CommFail" ){
			switch_str(act.policy){
				case_str(POLICY_AUTONOMY){
				}
				case_str(POLICY_MEDIUM){
					events->raiseEvent("/pauseMission");
				}
				case_str(POLICY_TELEOPERATION){
					events->raiseEvent("/goBack");
				}
			}
		}
		case_str( "NoPathFound" ){
			switch_str(act.policy){
				case_str(POLICY_AUTONOMY){

				}
				case_str(POLICY_MEDIUM){

				}
				case_str(POLICY_TELEOPERATION){

				}
			}
		}

		case_str( "ObstacleDetected" ){
			switch_str(act.policy){
				case_str(POLICY_AUTONOMY){

				}
				case_str(POLICY_MEDIUM){

				}
				case_str(POLICY_TELEOPERATION){

				}
			}
		}
		case_str( "RoadDetected" ){
			switch_str(act.policy){
				case_str(POLICY_AUTONOMY){

				}
				case_str(POLICY_MEDIUM){

				}
				case_str(POLICY_TELEOPERATION){

				}
			}
		}
		case_str( "Turn-over" ){
			switch_str(act.policy){
				case_str(POLICY_AUTONOMY){

				}
				case_str(POLICY_MEDIUM){

				}
				case_str(POLICY_TELEOPERATION){

				}
			}
		}
		case_str( "Collision" ){
			switch_str(act.policy){
				case_str(POLICY_AUTONOMY){

				}
				case_str(POLICY_MEDIUM){

				}
				case_str(POLICY_TELEOPERATION){

				}
			}
		}
		case_str( "NoGPS" ){
			switch_str(act.policy){
				case_str(POLICY_AUTONOMY){

				}
				case_str(POLICY_MEDIUM){

				}
				case_str(POLICY_TELEOPERATION){

				}
			}
		}
		case_str( "AssistanceRequired" ){
			switch_str(act.policy){
				case_str(POLICY_AUTONOMY){

				}
				case_str(POLICY_MEDIUM){

				}
				case_str(POLICY_TELEOPERATION){

				}
			}
		}
	}
}
void AblManager::on_deactivation(const Activated& act){
	decision_making::EventQueue* events = this->component->events();
	switch_str(act.trigger.name){
		case_str( "CommFail" ){
			switch_str(act.policy){
				case_str(POLICY_AUTONOMY){

				}
				case_str(POLICY_MEDIUM){
						events->raiseEvent("/resumeMission");
				}
				case_str(POLICY_TELEOPERATION){

				}
			}
		}
	}
}





