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

class MissionParams: public CallContextParameters{
public:
	ComponentMain* comp;
	MissionParams(ComponentMain* comp):comp(comp){}
	std::string str()const{return "";}
};

bool MissionLoaded;
FSM(MissionActive)
{
	FSM_STATES
	{
		MissionSpooling,
		MissionPaused,
		MissionAborted,
		MissionFinished,
	}
	FSM_START(MissionSpooling);
	FSM_BGN
	{
		FSM_STATE(MissionSpooling)
		{
			FSM_CALL_TASK(MissionSpooling)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/CompleteMission", FSM_NEXT(MissionFinished));
				FSM_ON_EVENT("/AbortMission", FSM_NEXT(MissionAborted));
				FSM_ON_EVENT("/PauseMission", FSM_NEXT(MissionPaused));
			}
		}
		FSM_STATE(MissionPaused)
		{
			FSM_CALL_TASK(MissionPaused)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/CompleteMission", FSM_NEXT(MissionFinished));
				FSM_ON_EVENT("/AbortMission", FSM_NEXT(MissionAborted));
				FSM_ON_EVENT("/ResumeMission", FSM_NEXT(MissionSpooling));
			}
		}
		FSM_STATE(MissionAborted)
		{
			FSM_CALL_TASK(MissionAborted)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/StartMission", FSM_NEXT(MissionSpooling));
			}
		}
		FSM_STATE(MissionFinished)
		{
			FSM_CALL_TASK(MissionFinished)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/StartMission", FSM_NEXT(MissionSpooling));
			}
		}
	}
	FSM_END
}

FSM(Mission_ON)
{
	FSM_STATES
	{
		NoMissionLoaded,
		MissionPending,
		MissionActive
	}
	FSM_START(NoMissionLoaded);
	FSM_BGN
	{
		FSM_STATE(NoMissionLoaded)
		{
			FSM_CALL_TASK(NoMissionLoaded)
			FSM_TRANSITIONS
			{
				FSM_ON_CONDITION(MissionLoaded, FSM_NEXT(MissionPending));
			}
		}
		FSM_STATE(MissionPending)
		{
			FSM_CALL_TASK(MissionPending)
			FSM_TRANSITIONS
			{
				//NOTE: It's not clear for transition from MissionPending to NoMissionLoaded
				FSM_ON_EVENT("/DeleteMission", FSM_NEXT(NoMissionLoaded));
				FSM_ON_EVENT("/ClearMissionBuffer", FSM_NEXT(NoMissionLoaded));
				FSM_ON_CONDITION(not MissionLoaded, FSM_NEXT(NoMissionLoaded));

				FSM_ON_EVENT("/StartMission", FSM_NEXT(MissionActive));
			}
		}
		FSM_STATE(MissionActive)
		{
			FSM_CALL_FSM(MissionActive)
			FSM_TRANSITIONS
			{
				//NOTE: It's not clear for transition from MissionActive to NoMissionLoaded
				FSM_ON_EVENT("/DeleteMission", FSM_NEXT(NoMissionLoaded));
				FSM_ON_EVENT("/Stendby", FSM_NEXT(NoMissionLoaded));
				FSM_ON_EVENT("/ClearMissionBuffer", FSM_NEXT(NoMissionLoaded));
				FSM_ON_CONDITION(not MissionLoaded, FSM_NEXT(NoMissionLoaded));
			}
		}

	}
	FSM_END
}

FSM(Mission)
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
				FSM_ON_EVENT("/SystemActivation", FSM_NEXT(ON));
			}
		}
		FSM_STATE(ON)
		{
			FSM_CALL_FSM(Mission_ON)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/PowerOff", FSM_NEXT(OFF));
			}
		}

	}
	FSM_END
}


TaskResult state_OFF(string id, const CallContext& context, EventQueue& events){
	PAUSE(10000);
	//diagnostic_msgs::DiagnosticStatus status;
	//context.parameters<MissionParams>.comp->publishDiagnostic(status);
	return TaskResult::SUCCESS();
}

TaskResult state_NoMissionLoaded(string id, const CallContext& context, EventQueue& events){
	PAUSE(10000);
	return TaskResult::SUCCESS();
}
TaskResult state_MissionPending(string id, const CallContext& context, EventQueue& events){
	PAUSE(10000);
	return TaskResult::SUCCESS();
}


TaskResult state_MissionSpooling(string id, const CallContext& context, EventQueue& events){
	PAUSE(10000);
	return TaskResult::SUCCESS();
}
TaskResult state_MissionPaused(string id, const CallContext& context, EventQueue& events){
	PAUSE(10000);
	return TaskResult::SUCCESS();
}
TaskResult state_MissionAborted(string id, const CallContext& context, EventQueue& events){
	PAUSE(10000);
	return TaskResult::SUCCESS();
}
TaskResult state_MissionFinished(string id, const CallContext& context, EventQueue& events){
	PAUSE(10000);
	return TaskResult::SUCCESS();
}

void startMission(ComponentMain* component){ 

	//ros_decision_making_init(argc, argv);
	RosEventQueue events;
	CallContext context;
	context.createParameters(new MissionParams(component));
	//events.async_spin();
	LocalTasks::registration("OFF",state_OFF);
	LocalTasks::registration("NoMissionLoaded",state_NoMissionLoaded);
	LocalTasks::registration("MissionPending",state_MissionPending);
	LocalTasks::registration("MissionSpooling",state_MissionSpooling);
	LocalTasks::registration("MissionPaused",state_MissionPaused);
	LocalTasks::registration("MissionAborted",state_MissionAborted);
	LocalTasks::registration("MissionFinished",state_MissionFinished);


	ROS_INFO("Starting smme (Mission)...");
	FsmMission(&context, &events);

}
