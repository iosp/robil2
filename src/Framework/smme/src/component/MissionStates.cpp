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
#include "MissionManager.h"

class MissionParams: public CallContextParameters{
public:
	ComponentMain* comp;
	string mission_id;
	MissionParams(ComponentMain* comp, string mission_id)
		:comp(comp), mission_id(mission_id)
	{

	}
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

FSM(Mission)
{
	FSM_STATES
	{
		MissionUnloaded,
		MissionPending,
		MissionActive
	}
	FSM_START(MissionPending);
	FSM_BGN
	{
		FSM_STATE(MissionUnloaded)
		{
			FSM_CALL_TASK(MissionUnloaded)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("MissionUnloaded/Stopped", FSM_STOP("Stopped",TaskResult::SUCCESS()));
			}
		}
		FSM_STATE(MissionPending)
		{
			FSM_CALL_TASK(MissionPending)
			FSM_TRANSITIONS
			{
				//NOTE: It's not clear for transition from MissionPending to NoMissionLoaded
				FSM_ON_EVENT("/DeleteMission", FSM_NEXT(MissionUnloaded));
				FSM_ON_EVENT("/ClearMissionBuffer", FSM_NEXT(MissionUnloaded));
				FSM_ON_CONDITION(not MissionLoaded, FSM_NEXT(MissionUnloaded));

				FSM_ON_EVENT("/StartMission", FSM_NEXT(MissionActive));
			}
		}
		FSM_STATE(MissionActive)
		{
			FSM_CALL_FSM(MissionActive)
			FSM_TRANSITIONS
			{
				//NOTE: It's not clear for transition from MissionActive to NoMissionLoaded
				FSM_ON_EVENT("/DeleteMission", FSM_NEXT(MissionUnloaded));
				FSM_ON_EVENT("/Stendby", FSM_NEXT(MissionUnloaded));
				FSM_ON_EVENT("/ClearMissionBuffer", FSM_NEXT(MissionUnloaded));
				FSM_ON_CONDITION(not MissionLoaded, FSM_NEXT(MissionUnloaded));
			}
		}

	}
	FSM_END
}

#define PARAMS \
		std::string mid = context.parameters<MissionParams>().mission_id;\
		ComponentMain* comp = context.parameters<MissionParams>().comp;

TaskResult state_MissionUnloaded(string id, const CallContext& context, EventQueue& events){
	PARAMS
	comp->mission_manager()->remove(mid);
	events.raiseEvent(Event("Stopped",context));
	return TaskResult::SUCCESS();
}
TaskResult state_MissionPending(string id, const CallContext& context, EventQueue& events){
	PARAMS
	comp->mission_manager()->mission_state("pending");
	return TaskResult::SUCCESS();
}

TaskResult state_MissionSpooling(string id, const CallContext& context, EventQueue& events){
	PARAMS
	comp->mission_manager()->change_mission(mid);
	comp->mission_manager()->mission_state("spooling");
	return TaskResult::SUCCESS();
}
TaskResult state_MissionPaused(string id, const CallContext& context, EventQueue& events){
	PARAMS
	comp->mission_manager()->mission_state("paused");
	return TaskResult::SUCCESS();
}
TaskResult state_MissionAborted(string id, const CallContext& context, EventQueue& events){
	PARAMS
	comp->mission_manager()->mission_state("aborted");
	return TaskResult::SUCCESS();
}
TaskResult state_MissionFinished(string id, const CallContext& context, EventQueue& events){
	PARAMS
	comp->mission_manager()->mission_state("Finished");
	return TaskResult::SUCCESS();
}

void initMissionTasks(){
	LocalTasks::registration("MissionUnloaded",state_MissionUnloaded);
	LocalTasks::registration("MissionPending",state_MissionPending);
	LocalTasks::registration("MissionSpooling",state_MissionSpooling);
	LocalTasks::registration("MissionPaused",state_MissionPaused);
	LocalTasks::registration("MissionAborted",state_MissionAborted);
	LocalTasks::registration("MissionFinished",state_MissionFinished);
}

void startMission(ComponentMain* component, std::string mission_id){

	//ros_decision_making_init(argc, argv);
	RosEventQueue events;
	CallContext context;
	context.push("mission_id");
	context.createParameters(new MissionParams(component, mission_id));
	//events.async_spin();


	ROS_INFO_STREAM("Starting smme (Mission:"<<mission_id<<")...");
	FsmMission(&context, &events);
	ROS_INFO_STREAM("Stop smme (Mission:"<<mission_id<<")...");
}

