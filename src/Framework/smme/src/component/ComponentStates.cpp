#include <iostream>
#include <ros/ros.h>
#include <decision_making/SynchCout.h>
#include <decision_making/BT.h>
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>
#include <decision_making/DebugModeTracker.hpp>

using namespace std;
using namespace decision_making;
#include "ComponentStates.h"
#include "AblManager.h"

class Params: public CallContextParameters{
public:
	ComponentMain* comp;
	Params(ComponentMain* comp):comp(comp){}
	std::string str()const{return "";}
};

FSM(smme_ON)
{
	FSM_STATES
	{
		Init,
		Ready,
		Emergency
	}
	FSM_START(Init);
	FSM_BGN
	{
		FSM_STATE(Init)
		{
			FSM_CALL_TASK(SYS_INIT)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/EndOfCoreSystemInit", FSM_NEXT(Ready));
			}
		}
		FSM_STATE(Ready)
		{
			FSM_CALL_TASK(SYS_READY)
			FSM_CALL_TASK(ABL)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/EStopCommand", FSM_NEXT(Emergency));
			}
		}
		FSM_STATE(Emergency)
		{
			FSM_CALL_TASK(SYS_EMERGENCY)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/ClearEStopCommand", FSM_NEXT(Ready));
			}
		}

	}
	FSM_END
}

FSM(smme)
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
			FSM_CALL_TASK(SYS_OFF)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/SystemActivation", FSM_NEXT(ON));
			}
		}
		FSM_STATE(ON)
		{
			FSM_CALL_FSM(smme_ON)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/PowerOff", FSM_NEXT(OFF));
			}
		}

	}
	FSM_END
}

namespace {

TaskResult state_OFF(string id, const CallContext& context, EventQueue& events){
	PAUSE(10000);
	//diagnostic_msgs::DiagnosticStatus status;
	//COMPONENT->publishDiagnostic(status);
	return TaskResult::SUCCESS();
}
TaskResult state_INIT(string id, const CallContext& context, EventQueue& events){
	//PAUSE(10000);
	events.raiseEvent(Event("/EndOfCoreSystemInit",context));
	return TaskResult::SUCCESS();
}
TaskResult state_READY(string id, const CallContext& context, EventQueue& events){
	PAUSE(10000);
	return TaskResult::SUCCESS();
}
TaskResult state_EMERGENCY(string id, const CallContext& context, EventQueue& events){
	PAUSE(10000);
	return TaskResult::SUCCESS();
}
TaskResult tsk_ABL(string id, const CallContext& context, EventQueue& events){
	AblManager abl(COMPONENT);
	abl.listen(&events);
	return TaskResult::SUCCESS();
}

}

void runComponent(int argc, char** argv, ComponentMain& component){

	ros_decision_making_init(argc, argv);
	startSystem(&component);

}
void startSystem(ComponentMain* component){

	RosEventQueue events;
	CallContext context;
	context.createParameters(new Params(component));
	//events.async_spin();
	LocalTasks::registration("SYS_OFF",state_OFF);
	LocalTasks::registration("SYS_INIT",state_INIT);
	LocalTasks::registration("SYS_READY",state_READY);
	LocalTasks::registration("SYS_EMERGENCY",state_EMERGENCY);
	LocalTasks::registration("ABL",tsk_ABL);

	ROS_INFO("Starting smme (System)...");
	Fsmsmme(&context, &events);

}
