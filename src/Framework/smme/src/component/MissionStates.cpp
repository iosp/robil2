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
	MissionParams():comp(0), mission_id(""){struct MissionParametersConstructorEmpty{}; throw MissionParametersConstructorEmpty();}
	MissionParams(ComponentMain* comp, string mission_id)
		:comp(comp), mission_id(mission_id)
	{

	}
	std::string str()const{return "";}
};

#define MISSION_ID string("/")+FSM_CONTEXT.parameters<MissionParams>().mission_id

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
				FSM_ON_EVENT(MISSION_ID+"/CompleteMission", FSM_NEXT(MissionFinished));
				FSM_ON_EVENT(MISSION_ID+"/AbortMission", FSM_NEXT(MissionAborted));
				FSM_ON_EVENT(MISSION_ID+"/PauseMission", FSM_NEXT(MissionPaused));
			}
		}
		FSM_STATE(MissionPaused)
		{
			FSM_CALL_TASK(MissionPaused)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT(MISSION_ID+"/CompleteMission", FSM_NEXT(MissionFinished));
				FSM_ON_EVENT(MISSION_ID+"/AbortMission", FSM_NEXT(MissionAborted));

				FSM_ON_EVENT(MISSION_ID+"/ResumeMission", FSM_RAISE("/RestartTask"));
				FSM_ON_EVENT(MISSION_ID+"/ResumeMission", FSM_NEXT(MissionSpooling));
			}
		}
		FSM_STATE(MissionAborted)
		{
			FSM_CALL_TASK(MissionAborted)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT(MISSION_ID+"/StartMission", FSM_NEXT(MissionSpooling));
			}
		}
		FSM_STATE(MissionFinished)
		{
			FSM_CALL_TASK(MissionFinished)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT(MISSION_ID+"/StartMission", FSM_NEXT(MissionSpooling));
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
	call_ctx.pop();
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
				FSM_ON_EVENT(MISSION_ID+"/DeleteMission", FSM_NEXT(MissionUnloaded));
				FSM_ON_EVENT("/ClearMissionBuffer", FSM_NEXT(MissionUnloaded));
				FSM_ON_EVENT(MISSION_ID+"/StartMission", FSM_NEXT(MissionActive));
			}
		}
		FSM_STATE(MissionActive)
		{
			FSM_CALL_FSM(MissionActive)
			FSM_TRANSITIONS
			{
				//NOTE: It's not clear for transition from MissionActive to NoMissionLoaded
				FSM_ON_EVENT(MISSION_ID+"/DeleteMission", FSM_NEXT(MissionUnloaded));
				FSM_ON_EVENT(MISSION_ID+"/Standby", FSM_NEXT(MissionUnloaded));
				FSM_ON_EVENT(MISSION_ID+"/ClearMissionBuffer", FSM_NEXT(MissionUnloaded));
			}
		}

	}
	FSM_END
}

#define PARAMS \
		std::string mid = context.parameters<MissionParams>().mission_id;\
		ComponentMain* comp = context.parameters<MissionParams>().comp;
#define MM comp->mission_manager()

TaskResult state_MissionUnloaded(string id, const CallContext& context, EventQueue& events){
	PARAMS
	MM->remove(mid);
	events.raiseEvent(Event("Stopped",context));
	return TaskResult::SUCCESS();
}
TaskResult state_MissionPending(string id, const CallContext& context, EventQueue& events){
	PARAMS
	MM->mission_state("pending");
	return TaskResult::SUCCESS();
}

TaskResult state_MissionSpooling(string id, const CallContext& context, EventQueue& events){
	PARAMS
	MM->change_mission(mid);
	MM->mission_state("spooling");
	events.raiseEvent(Event("/StartTask",context));
	return TaskResult::SUCCESS();
}
TaskResult state_MissionPaused(string id, const CallContext& context, EventQueue& events){
	PARAMS
	MM->mission_state("paused");
	return TaskResult::SUCCESS();
}
TaskResult state_MissionAborted(string id, const CallContext& context, EventQueue& events){
	PARAMS
	MM->mission_state("aborted");
	events.raiseEvent(Event("/StopTask",context));
	return TaskResult::SUCCESS();
}
TaskResult state_MissionFinished(string id, const CallContext& context, EventQueue& events){
	PARAMS
	MM->mission_state("finished");
	events.raiseEvent(Event("/StopTask",context));
	return TaskResult::SUCCESS();
}

#include <robil_msgs/MissionState.h>
MissionManager* __mission_manager=0;
bool get_mission_state(robil_msgs::MissionState::Request& req,robil_msgs::MissionState::Response& res){
	if(__mission_manager){
		std::string state = __mission_manager->print_state();
		res.states = state;
		return true;
	}
	return false;

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
	context.push(mission_id);
	context.createParameters(new MissionParams(component, mission_id));
	//events.async_spin();


	ROS_INFO_STREAM("Starting smme (Mission:"<<mission_id<<")...");
	FsmMission(&context, &events);
	ROS_INFO_STREAM("Stop smme (Mission:"<<mission_id<<")...");
}

