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
FSM(shiffon2ros_ON)
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
				FSM_ON_EVENT("/EndOfInit", FSM_NEXT(READY));
			}
		}
		FSM_STATE(READY)
		{
			FSM_CALL_TASK(READY)
			FSM_TRANSITIONS{
				FSM_ON_EVENT("/shiffon2ros/Standby", FSM_NEXT(STANDBY));
			}
		}
		FSM_STATE(STANDBY)
		{
			FSM_CALL_TASK(STANDBY)
			FSM_TRANSITIONS{
				FSM_ON_EVENT("/shiffon2ros/Resume", FSM_NEXT(READY));
			}
		}

	}
	FSM_END
}

FSM(shiffon2ros)
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
				FSM_ON_EVENT("/shiffon2ros/Activation", FSM_NEXT(ON));
			}
		}
		FSM_STATE(ON)
		{
			FSM_CALL_FSM(shiffon2ros_ON)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/Shutdown", FSM_NEXT(OFF));
				FSM_ON_EVENT("/shiffon2ros/Shutdown", FSM_NEXT(OFF));
			}
		}
	}
	FSM_END
}

TaskResult state_OFF(string id, const CallContext& context, EventQueue& events){
	PAUSE(10000);
	return TaskResult::SUCCESS();
}


TaskResult state_INIT(string id, const CallContext& context, EventQueue& events){
	ROS_INFO("shiffon2ros Init !!");

	COMPONENT->InitShiphonConection();
	PAUSE(300);

	Event e("EndOfInit");
	events.raiseEvent(e);
	return TaskResult::SUCCESS();
}

TaskResult state_READY(string id, const CallContext& context, EventQueue& events){
	ROS_INFO("shiffon2ros Ready !!");

	ros::Rate IPON_rate(100);
	while (ros::ok()) {
        ros::Time now = ros::Time::now();
        COMPONENT->ReadAndPub_ShiphonGPS(now);
        COMPONENT->ReadAndPub_ShiphonINS(now);
        COMPONENT->ReadAndPub_ShiphonGpsSpeed(now);
		IPON_rate.sleep();
	}
	return TaskResult::SUCCESS();
}

TaskResult state_STANDBY(string id, const CallContext& context, EventQueue& events){
	PAUSE(10000);
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

	ROS_INFO("Starting shiffon2ros...");
	Fsmshiffon2ros(&context, &events);


	Shiphon_Ctrl * 	_shiphonCtrl;

}
