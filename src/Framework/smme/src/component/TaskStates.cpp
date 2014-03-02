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

class TaskParams: public CallContextParameters{
public:
	ComponentMain* comp;
	TaskParams(ComponentMain* comp)
		:comp(comp)
	{

	}
	std::string str()const{return "";}
};

bool MissionLoaded;
FSM(TaskActive)
{
	FSM_STATES
	{
		TaskSpooling,
		TaskPaused,
		TaskAborted,
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
				FSM_ON_EVENT("/CompleteTask", FSM_NEXT(TaskFinished));
				FSM_ON_EVENT("/AbortTask", FSM_NEXT(TaskAborted));
				FSM_ON_EVENT("/PauseTask", FSM_NEXT(TaskPaused));
			}
		}
		FSM_STATE(TaskPaused)
		{
			FSM_CALL_TASK(TaskPaused)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/CompleteTask", FSM_NEXT(TaskFinished));
				FSM_ON_EVENT("/AbortTask", FSM_NEXT(TaskAborted));
				FSM_ON_EVENT("/ResumeTask", FSM_NEXT(TaskSpooling));
			}
		}
		FSM_STATE(TaskAborted)
		{
			FSM_CALL_TASK(TaskAborted)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/StartTask", FSM_NEXT(TaskSpooling));
			}
		}
		FSM_STATE(TaskFinished)
		{
			FSM_CALL_TASK(TaskFinished)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/StartTask", FSM_NEXT(TaskSpooling));
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
	FSM_BGN
	{
		FSM_STATE(TaskPending)
		{
			FSM_CALL_TASK(TaskPending)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/StartTask", FSM_NEXT(TaskActive));
			}
		}
		FSM_STATE(TaskActive)
		{
			FSM_CALL_FSM(TaskActive)
			FSM_TRANSITIONS
			{
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

TaskResult state_TaskPending(string id, const CallContext& context, EventQueue& events){
	PAUSE(10000);
	return TaskResult::SUCCESS();
}


TaskResult state_TaskSpooling(string id, const CallContext& context, EventQueue& events){
	PAUSE(10000);
	return TaskResult::SUCCESS();
}
TaskResult state_TaskPaused(string id, const CallContext& context, EventQueue& events){
	PAUSE(10000);
	return TaskResult::SUCCESS();
}
TaskResult state_TaskAborted(string id, const CallContext& context, EventQueue& events){
	PAUSE(10000);
	return TaskResult::SUCCESS();
}
TaskResult state_TaskFinished(string id, const CallContext& context, EventQueue& events){
	PAUSE(10000);
	return TaskResult::SUCCESS();
}


void startTask(ComponentMain* component){

	//ros_decision_making_init(argc, argv);
	RosEventQueue events;
	CallContext context;
	context.createParameters(new TaskParams(component));
	//events.async_spin();
	LocalTasks::registration("OFF",state_OFF);
	LocalTasks::registration("TaskPending",state_MissionPending);
	LocalTasks::registration("TaskSpooling",state_MissionSpooling);
	LocalTasks::registration("TaskPaused",state_MissionPaused);
	LocalTasks::registration("TaskAborted",state_MissionAborted);
	LocalTasks::registration("TaskFinished",state_MissionFinished);


	ROS_INFO("Starting smme (Task)...");
	FsmMission(&context, &events);

}
