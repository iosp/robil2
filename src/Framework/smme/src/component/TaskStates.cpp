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
#include "Types.h"


class TaskParams: public CallContextParameters{
public:
	ComponentMain* comp;
	string mission_id;
	TaskParams():comp(0), mission_id(""){struct TaskParamsConstructorEmpty{}; throw TaskParamsConstructorEmpty();}
	TaskParams(ComponentMain* comp, string mission_id)
		:comp(comp), mission_id(mission_id)
	{

	}
	std::string str()const{return "";}
};

#define MID_PREF(X) string("/mission/")+X
#define MISSION_ID MID_PREF(FSM_CONTEXT.parameters<TaskParams>().mission_id)


FSM(TaskActive)
{
	FSM_STATES
	{
		TaskSpooling,
		TaskPaused,
		TaskAborted,
//		TaskTryNext,
		TaskFinished,
	}
	FSM_START(TaskSpooling);
	FSM_BGN
	{
		FSM_STATE(TaskSpooling)
		{
			FSM_CALL_TASK(TaskSpooling)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT(MISSION_ID+"/CompleteTask", FSM_NEXT(TaskFinished));
				FSM_ON_EVENT(MISSION_ID+"/AbortTask", FSM_NEXT(TaskAborted));
				FSM_ON_EVENT(MISSION_ID+"/PauseTask", FSM_NEXT(TaskPaused));
			}
		}
		FSM_STATE(TaskPaused)
		{
			FSM_CALL_TASK(TaskPaused)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT(MISSION_ID+"/CompleteTask", FSM_NEXT(TaskFinished));
				FSM_ON_EVENT(MISSION_ID+"/AbortTask", FSM_NEXT(TaskAborted));
				FSM_ON_EVENT(MISSION_ID+"/ResumeTask", FSM_NEXT(TaskSpooling));
			}
		}
		FSM_STATE(TaskAborted)
		{
			FSM_RAISE(MISSION_ID+"/AbortMission")
			FSM_CALL_TASK(TaskAborted)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT(MISSION_ID+"/StartTask", FSM_NEXT(TaskSpooling));
			}
		}
//		FSM_STATE(TaskTryNext)
//		{
//			FSM_CALL_TASK(TaskFinished)
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT("TaskFinished/restart", FSM_NEXT(TaskSpooling));
//				FSM_ON_EVENT("TaskFinished/complete", FSM_NEXT(TaskFinished));
//			}
//		}
		FSM_STATE(TaskFinished)
		{
			FSM_CALL_TASK(TaskFinished)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT(MISSION_ID+"/StartTask", FSM_NEXT(TaskSpooling));
			}
		}
	}
	FSM_END
}

FSM(Task)
{
	FSM_STATES
	{
		TaskPending,
		TaskActive,
	}
	FSM_START(TaskPending);
	call_ctx.pop();
	FSM_BGN
	{
		FSM_STATE(TaskPending)
		{
			FSM_CALL_TASK(TaskPending)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT(MISSION_ID+"/StartTask", FSM_NEXT(TaskActive));
			}
		}
		FSM_STATE(TaskActive)
		{
			FSM_CALL_FSM(TaskActive)
			FSM_TRANSITIONS
			{
				//FSM_ON_EVENT(MISSION_ID+"/StopTask", FSM_RAISE(MISSION_ID+"/CompleteTask"));
				FSM_ON_EVENT(MISSION_ID+"/StopTask", FSM_NEXT(TaskPending));
				FSM_ON_EVENT(MISSION_ID+"/RestartTask", FSM_NEXT(TaskActive));
			}
		}
	}
	FSM_END
}

#define PARAMS \
		std::string mid = context.parameters<TaskParams>().mission_id;\
		ComponentMain* comp = context.parameters<TaskParams>().comp;
#define MM comp->mission_manager()

TaskResult state_TaskPending(string id, const CallContext& context, EventQueue& events){
	PARAMS
	MissionManager::MissionID cmid = MM->get_current_mission().mid;
	MM->change_mission(mid);
	MM->task_state("pending");
	MM->change_mission(cmid);
	return TaskResult::SUCCESS();
}

TaskResult state_TaskSpooling(string id, const CallContext& context, EventQueue& events){
	PARAMS
	MM->task_state("spooling");
	if(MM->task_type()==MissionManager::TT_Navigation){
		MissionManager::NavTask task = MM->get_nav_task();
		config::SMME::pub::GlobalPath path = extract_path(task);
		events.raiseEvent("/pp/Resume");
		comp->publishGlobalPath(path);
	}else{
		MissionManager::ManTask task = MM->get_man_task();
		events.raiseEvent("/wsm/Resume");
		comp->publishWorkSeqData(task);
	}
	return TaskResult::SUCCESS();
}
TaskResult state_TaskPaused(string id, const CallContext& context, EventQueue& events){
	PARAMS
	MM->task_state("paused");
	if(MM->task_type()==MissionManager::TT_Navigation)events.raiseEvent("/pp/Standby");
	else events.raiseEvent("/wsm/Standby");
	return TaskResult::SUCCESS();
}
TaskResult state_TaskAborted(string id, const CallContext& context, EventQueue& events){
	PARAMS
	MM->task_state("aborted");
	if(MM->task_type()==MissionManager::TT_Navigation)events.raiseEvent("/pp/Standby");
	else events.raiseEvent("/wsm/Standby");
	return TaskResult::SUCCESS();
}
TaskResult state_TaskFinished(string id, const CallContext& context, EventQueue& events){
	PARAMS
	MM->task_state("finished");
	if(MM->task_type()==MissionManager::TT_Navigation)events.raiseEvent("/pp/Standby");
	else events.raiseEvent("/wsm/Standby");
//	if( MM->next_task() ){
//		events.raiseEvent(Event("restart",context));
//	}else{
//		events.raiseEvent(Event("complete",context));
//	}
	return TaskResult::SUCCESS();
}


#include <robil_msgs/MissionState.h>
MissionManager* __mission_manager=0;
bool service_get_mission_state(robil_msgs::MissionState::Request& req,robil_msgs::MissionState::Response& res){
	if(__mission_manager){
		std::string state = __mission_manager->print_state();
		res.states = state;
		return true;
	}
	return false;
}

ros::ServiceServer ss_get_mission_state;
void startStateService(ComponentMain* component, std::string mid){
	__mission_manager = component->mission_manager();
	__mission_manager->change_mission(mid);
	ros::NodeHandle node;
	ss_get_mission_state = node.advertiseService("/mission_state",&service_get_mission_state);
}


void initTask(){
	LocalTasks::registration("TaskPending",state_TaskPending);
	LocalTasks::registration("TaskSpooling",state_TaskSpooling);
	LocalTasks::registration("TaskPaused",state_TaskPaused);
	LocalTasks::registration("TaskAborted",state_TaskAborted);
	LocalTasks::registration("TaskFinished",state_TaskFinished);
}

void TaskMachine::startTask(ComponentMain* component, std::string mission_id){

	RosEventQueue events;
	events_ptr = &events;
	CallContext context;
	context.push("mission");
	context.push(mission_id);
	context.createParameters(new TaskParams(component, mission_id));

	ROS_INFO_STREAM("Starting smme (Task:"<<mission_id<<")...");
	FsmTask(&context, &events);
	ROS_INFO_STREAM("Stop smme (Task:"<<mission_id<<")");

}
void TaskMachine::stop(){
	static_cast<RosEventQueue*>(events_ptr)->close();
	thread.join_all();
}
