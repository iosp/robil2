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

#define MID_PREF(X) string("/mission/")+X
#define MISSION_ID MID_PREF(FSM_CONTEXT.parameters<MissionParams>().mission_id)

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
			FSM_RAISE(MISSION_ID+"/StartTask")
			FSM_RAISE(MISSION_ID+"/ResumeTask")
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
			//FSM_RAISE(MISSION_ID+"/PauseTask")
			FSM_RAISE(MISSION_ID+"/StopTask")
			FSM_CALL_TASK(MissionPaused)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT(MISSION_ID+"/CompleteMission", FSM_NEXT(MissionFinished));
				FSM_ON_EVENT(MISSION_ID+"/AbortMission", FSM_NEXT(MissionAborted));
				FSM_ON_EVENT(MISSION_ID+"/ResumeMission", FSM_NEXT(MissionSpooling));
			}
		}
		FSM_STATE(MissionAborted)
		{
			FSM_RAISE(MISSION_ID+"/StopTask")
			FSM_CALL_TASK(MissionAborted)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT(MISSION_ID+"/StartMission", FSM_NEXT(MissionSpooling));
			}
		}
		FSM_STATE(MissionFinished)
		{
			FSM_RAISE(MISSION_ID+"/StopTask")
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
			FSM_RAISE(MISSION_ID+"/StopTask");
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

bool extend_events_names(Event& e, std::string mid_pref, EventQueue& events){
#		define EXTEND(NAME) \
		if(e == Event(NAME)){\
			events.raiseEvent(Event(mid_pref+NAME));\
			return true;\
		}
		//----------- TASK GLOBAL EVENT -------------
		EXTEND("/CompleteTask")
		EXTEND("/AbortTask")
		//----------- MISSION GLOBAL EVENT ----------
		EXTEND("/CompleteMission")
		EXTEND("/PauseMission")
		EXTEND("/AbortMission")
		EXTEND("/ResumeMission")
		return false;
#		undef EXTEND
}

TaskResult state_MissionUnloaded(string id, const CallContext& context, EventQueue& events){
	PARAMS
	MM->remove(mid);
	events.raiseEvent(Event("Stopped",context));
	return TaskResult::SUCCESS();
}
TaskResult state_MissionPending(string id, const CallContext& context, EventQueue& events){
	PARAMS
	MissionManager::MissionID cmid = MM->get_current_mission().mid;
	MM->change_mission(mid);
	MM->mission_state("pending");
	MM->change_mission(cmid);
	return TaskResult::SUCCESS();
}

TaskResult state_MissionSpooling(string id, const CallContext& context, EventQueue& events){
	PARAMS
	MM->change_mission(mid);
	MM->mission_state("spooling");
	while(events.isTerminated()==false and ros::ok()){
		Event e = events.waitEvent();
		extend_events_names(e, MID_PREF(mid), events);
		if(e == Event(MID_PREF(mid)+"/CompleteTask")){
			if( MM->next_task() ){
				this_thread::sleep(milliseconds(100));
				events.raiseEvent(MID_PREF(mid)+"/StartTask");
			}else{
				events.raiseEvent(MID_PREF(mid)+"/CompleteMission");
			}
		}
	}
	return TaskResult::SUCCESS();
}
TaskResult state_MissionPaused(string id, const CallContext& context, EventQueue& events){
	PARAMS
	MM->mission_state("paused");
	while(events.isTerminated()==false and ros::ok()){
		Event e = events.waitEvent();
		//extend_events_names(e, MID_PREF(mid), events);
	}
	return TaskResult::SUCCESS();
}
TaskResult state_MissionAborted(string id, const CallContext& context, EventQueue& events){
	PARAMS
	MM->mission_state("aborted");
	return TaskResult::SUCCESS();
}
TaskResult state_MissionFinished(string id, const CallContext& context, EventQueue& events){
	PARAMS
	MM->mission_state("finished");
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

MissionMachine::MissionMachine(ComponentMain* comp,std::string mid)
:
		component(comp),mission_id(mid),task(comp,mid),events_ptr(0)
{
	static int _init=init(comp,mid);
	manager = component->mission_manager();
	manager->start_mission(mid);
}


void MissionMachine::startMission(ComponentMain* component, std::string mission_id){

	RosEventQueue events;
	events_ptr = &events;
	CallContext context;
	context.push("mission");
	context.push(mission_id);
	context.createParameters(new MissionParams(component, mission_id));

	ROS_INFO_STREAM("Starting smme (Mission:"<<mission_id<<")...");
	FsmMission(&context, &events);
	ROS_INFO_STREAM("Stop smme (Mission:"<<mission_id<<")");
}
void MissionMachine::stop(){
	task.stop();
	static_cast<RosEventQueue*>(events_ptr)->close();
	thread.join_all();
}
