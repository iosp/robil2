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
FSM(lli_ON)
{
	FSM_STATES
	{
		INIT,
		READY,
		STANDBY
	}
	FSM_START(INIT);
	FSM_BGN
	{
		FSM_STATE(INIT)
		{
			FSM_CALL_TASK(INIT)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/EndOfInit", FSM_NEXT(STANDBY));
			}
		}
		FSM_STATE(READY)
		{
			FSM_CALL_TASK(READY)
			FSM_TRANSITIONS{
				FSM_ON_EVENT("/lli/Standby", FSM_NEXT(STANDBY));
			}
		}
		FSM_STATE(STANDBY)
		{
			FSM_CALL_TASK(STANDBY)
			FSM_TRANSITIONS{
				FSM_ON_EVENT("/lli/Resume", FSM_NEXT(READY));
				FSM_ON_EVENT("/lli_ready", FSM_NEXT(READY));
			}
		}

	}
	FSM_END
}

FSM(lli)
{
	FSM_STATES
	{
		OFF,
		ON
	}
	FSM_START(ON);
	FSM_BGN
	{
		FSM_STATE(OFF)
		{
			FSM_CALL_TASK(OFF)
			FSM_TRANSITIONS
			{	
				FSM_ON_EVENT("/Activation", FSM_NEXT(ON));
				FSM_ON_EVENT("/lli/Activation", FSM_NEXT(ON));
			}
		}
		FSM_STATE(ON)
		{
			FSM_CALL_FSM(lli_ON)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/Shutdown", FSM_NEXT(OFF));
				FSM_ON_EVENT("/lli/Shutdown", FSM_NEXT(OFF));
			}
		}

	}
	FSM_END
}

TaskResult state_OFF(string id, const CallContext& context, EventQueue& events){
	COMPONENT->releaseDriverAndManipulator();
	return TaskResult::SUCCESS();
}
TaskResult state_INIT(string id, const CallContext& context, EventQueue& events){
	int counter=0;
	COMPONENT->workerFunc();
	ros::Duration oneSec(1.0);
	oneSec.sleep();
	while (COMPONENT->IsCLLIStillInInit()){
		if (counter > 10000) break;
		oneSec.sleep();
		counter++;
	}
	if (counter > 10000){
		printf("LLI STOPPED WO INITIALIZATION COMPLETED\n");
		Event e("/lli/Shutdown");
		events.raiseEvent(e);
		return TaskResult::FAIL();
	}

	Event e("EndOfInit");
	events.raiseEvent(e);
	return TaskResult::SUCCESS();
}
TaskResult state_READY(string id, const CallContext& context, EventQueue& events){
	if(!ros::ok()){			/* checks whether the node failed */
		ROS_INFO("LLI STOPPED");
		Event e("/lli/Shutdown");
		events.raiseEvent(e);
		return TaskResult::TERMINATED();
	}

	return TaskResult::SUCCESS();
}

TaskResult state_STANDBY(string id, const CallContext& context, EventQueue& events){
	ros::Rate r(10);
	COMPONENT->setReady();
	COMPONENT->checkReady();
	while (COMPONENT->StateNotReady()){
		COMPONENT->checkReady();
	}
	Event e("/lli_ready");
	events.raiseEvent(e);
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

	ROS_INFO("Starting lli...");
	Fsmlli(&context, &events);
	ROS_INFO("After Starting lli...");

}
