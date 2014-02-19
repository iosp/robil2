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

FSM(Monitoring_ON)
{
	FSM_STATES
	{
		INIT,
		READY
	}
	FSM_START(INIT);
	FSM_BGN
	{
		FSM_STATE(INIT)
		{
			FSM_CALL_TASK(INIT)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/EndOfInit", FSM_NEXT(READY));
			}
		}
		FSM_STATE(READY)
		{
			FSM_CALL_TASK(READY)
			FSM_TRANSITIONS{}
		}

	}
	FSM_END
}

FSM(Monitoring)
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
			}
		}
		FSM_STATE(ON)
		{
			FSM_CALL_FSM(Monitoring_ON)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/Shutdown", FSM_NEXT(OFF));
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
	PAUSE(10000);
	return TaskResult::SUCCESS();
}
TaskResult state_READY(string id, const CallContext& context, EventQueue& events){
	PAUSE(10000);
	return TaskResult::SUCCESS();
}

}

void init_dm(int argc, char** argv){
	ros_decision_making_init(argc, argv);
}
void startComponent(ComponentMain* component){
	boost::this_thread::sleep(boost::posix_time::seconds(1));
	RosEventQueue events;
	CallContext context;
	context.createParameters(new Params(component));
	//events.async_spin();
	LocalTasks::registration("OFF",state_OFF);
	LocalTasks::registration("INIT",state_INIT);
	LocalTasks::registration("READY",state_READY);

	ROS_INFO("Starting ssm (Monitoring)...");
	FsmMonitoring(&context, &events);
}
