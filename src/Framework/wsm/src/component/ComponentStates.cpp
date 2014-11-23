#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <decision_making/SynchCout.h>
#include <decision_making/BT.h>
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>
#include <decision_making/DebugModeTracker.hpp>
#include <robil_msgs/Map.h>
#include <robil_msgs/MapCell.h>

using namespace std;
using namespace decision_making;

#include "ComponentStates.h"
#include "WsmTask.h"

ComponentMain *Global_comp ;
bool pause_time = false;

bool SensorConnection;
sensor_msgs::JointState jointStates;

class Params: public CallContextParameters{
public:
	ComponentMain* comp;
	Params(ComponentMain* comp):comp(comp){}
	std::string str()const{return "";}
};

void JointStatesCallback(const sensor_msgs::JointStateConstPtr &msg)
{
	jointStates = sensor_msgs::JointState(*msg);
	Global_comp->jointStates = &jointStates;
}

void pauseCallback(const std_msgs::StringConstPtr &msg)
{
		if((msg->data.find("ResumeTask",0) != -1)&&(pause_time)){
			pause_time = false ;
			if(Global_comp->cur_mission == NULL){
			//	ROS_ERROR("Not task to resume/pause");
				return;
			}
			if((Global_comp->cur_mission->Get_status() == "paused")){
				Global_comp->cur_mission->Set_task_status("active");
				return;
			}
			else{
				ROS_ERROR("No Task to resume, has Task %d at status %s",Global_comp->cur_mission->Get_Task_id(),Global_comp->cur_mission->Get_status().c_str());
				return;
			}
		}
		if(Global_comp->cur_mission == NULL){
					//ROS_ERROR("Not task to resume/pause");
					return;
				}
		if((msg->data.find("PauseMission",0) != -1)&&(Global_comp->cur_mission->Get_status()=="active")){
			pause_time = true ;
			Global_comp->cur_mission->Set_task_status("paused");
			return;
		}
		else{
			//ROS_ERROR("No Task to pause, has Task %d at status %s",Global_comp->cur_mission->Get_Task_id(),Global_comp->cur_mission->Get_status().c_str());
			return;
		}
		return;
}

FSM(wsm_WORK)
{
	FSM_STATES
	{
		STANDBY,
		READY
	}
	FSM_START(STANDBY);
	FSM_BGN
	{
		FSM_STATE(STANDBY)
		{
			FSM_CALL_TASK(STANDBY);
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/wsm/Resume", FSM_NEXT(READY));
			}
		}
		FSM_STATE(READY)
		{
			FSM_CALL_TASK(READY);
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/wsm/Standby", FSM_NEXT(STANDBY));
			}
		}

	}
	FSM_END
}

FSM(wsm_ON)
{
	FSM_STATES
	{
		INIT,
		WORK
	}
	FSM_START(INIT);
	FSM_BGN
	{
		FSM_STATE(INIT)
		{
			FSM_CALL_TASK(INIT);
			FSM_TRANSITIONS
			{
				//FSM_ON_CONDITION(SensorConnection, FSM_NEXT(WORK));
				FSM_ON_EVENT("/wsm/SensorConnected", FSM_NEXT(WORK));
			}
		}
		FSM_STATE(WORK)
		{
			FSM_CALL_FSM(wsm_WORK)
			FSM_TRANSITIONS
			{
				//FSM_ON_CONDITION(not SensorConnection, FSM_NEXT(INIT));
				FSM_ON_EVENT("/wsm/SensorNotConnected", FSM_NEXT(INIT));
			}
		}

	}
	FSM_END
}

FSM(wsm)
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
			FSM_CALL_TASK(OFF);
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/Activation", FSM_NEXT(ON));
				FSM_ON_EVENT("/wsm/Activation", FSM_NEXT(ON));
			}
		}
		FSM_STATE(ON)
		{
			FSM_CALL_FSM(wsm_ON)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/Shutdown", FSM_NEXT(OFF));
				FSM_ON_EVENT("/wsm/Shutdown", FSM_NEXT(OFF));
			}
		}

	}
	FSM_END
}

TaskResult state_OFF(string id, const CallContext& context, EventQueue& events){

	//diagnostic_msgs::DiagnosticStatus status;
	//COMPONENT->publishDiagnostic(status);
	ROS_INFO("WSM OFF");
	return TaskResult::SUCCESS();
}

TaskResult state_INIT(string id, const CallContext& context, EventQueue& events){
	//PAUSE(10000);
	ROS_INFO("WSM INIT");
	while(COMPONENT->receivedLocation == NULL){}
	COMPONENT->z_offset = COMPONENT->receivedLocation->pose.pose.position.x ;
	ROS_INFO("Initial ground offset is: %g",COMPONENT->z_offset );
	events.raiseEvent(Event("/wsm/SensorConnected"));
	return TaskResult::SUCCESS();
}

TaskResult state_READY(string id, const CallContext& context, EventQueue& events){

	ROS_INFO("WSM At Ready");

	while(1)
	{
		if(events.isTerminated() || !ros::ok()){			/* checks whether the line is empty, or node failed */
			ROS_INFO("STOPPED");
			return TaskResult::TERMINATED();
		}

		while(COMPONENT->cur_mission == NULL){
			PAUSE(10000);
			sleep(5);
			ROS_INFO("No new Task");
		}
		while(COMPONENT->cur_mission->Get_status() == "active"){
			ROS_INFO("Executing steps");
			COMPONENT->cur_mission->publish_step_diag(1,0);
			COMPONENT->cur_mission->execute_next_step();
			COMPONENT->cur_mission->Update_step();
		}
		if(COMPONENT->cur_mission->Get_status() == "complete")
		{
			events.raiseEvent(Event("/CompleteTask"));
			ROS_INFO("Mission complete");
			delete COMPONENT->cur_mission ;
			COMPONENT->cur_mission = NULL;
		}
    }

	return TaskResult::SUCCESS();
}

TaskResult state_STANDBY(string id, const CallContext& context, EventQueue& events){
	ROS_INFO("WSM_STANDBY");
	while(pause_time)
	{
		ROS_INFO("I'm in pause mode..");
		PAUSE(1000);
	}
	events.raiseEvent(Event("/wsm/Resume",context));
	return TaskResult::SUCCESS();
}

void runComponent(int argc, char** argv, ComponentMain& component){

	ros::NodeHandle n;
	ros::Subscriber jointstatesSub = n.subscribe<sensor_msgs::JointState>("/Sahar/joint_states", 100, &JointStatesCallback);
	ros::Subscriber PauseMission = n.subscribe<std_msgs::String>("/decision_making/events" , 100 , &pauseCallback);

	Global_comp = &component ;

	ros_decision_making_init(argc, argv);
	RosEventQueue events;
	CallContext context;
	context.createParameters(new Params(&component));
	//events.async_spin();

	LocalTasks::registration("OFF",state_OFF);
	LocalTasks::registration("INIT",state_INIT);
	LocalTasks::registration("READY",state_READY);
	LocalTasks::registration("STANDBY",state_STANDBY);
//	event_queue = &events ;

	//ROS_INFO("Starting wsm (WorkSequnceManager)...");
	//ROS_INFO("WSM AT FSM1");
	Fsmwsm(&context, &events);
	//ROS_INFO("WSM AT FSM2");
}







