
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

#define PUSH_KEYVALUE(oObject, kKey, vValue) {diagnostic_msgs::KeyValue kv; \
											kv.key = kKey; \
											kv.value = vValue; \
											oObject.values.push_back(kv); }

class Params: public CallContextParameters{
public:
	ComponentMain* comp;
	Params(ComponentMain* comp):comp(comp){}
	std::string str()const{return "";}
};

bool SensorConnection;
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
	events.raiseEvent(Event("/wsm/SensorConnected"));
	return TaskResult::SUCCESS();
}
TaskResult state_READY(string id, const CallContext& context, EventQueue& events){

#define WSM_USE_LOCALIZATION

	ros::Time tic = ros::Time::now();
	ros::Time toc;
	double value;
	config::WSM::sub::BladePosition * presentBladePos = NULL;
	config::WSM::sub::WorkSeqData * presentWorkSeq = NULL;

	uint64_t counter;

	ROS_INFO("WSM At Ready");

	while(1){
		if(events.isTerminated() || !ros::ok()){			/* checks whether the line is empty, or node failed */
			ROS_INFO("STOPPED");
			return TaskResult::TERMINATED();
		}

		if(COMPONENT->receivedWorkSeqData == NULL){
			PAUSE(1000);
			continue;
		}

		//Check for a new task
		if(presentWorkSeq == NULL){
			presentWorkSeq = COMPONENT->receivedWorkSeqData;
			COMPONENT->receivedWorkSeqData = NULL;
		}else{
			if(COMPONENT->receivedWorkSeqData->task_id != presentWorkSeq->task_id){
				delete presentWorkSeq;
				presentWorkSeq = COMPONENT->receivedWorkSeqData;
				COMPONENT->receivedWorkSeqData = NULL;
			}else{
				//What? new task id is the same as the old one? undefined behavior

			}
		}

		//We should do something with these fields:
		//presentWorkSeq->task_id
		//presentWorkSeq->task_description

		ROS_INFO("Got task %s", presentWorkSeq->task_description.c_str());
		for(std::vector<robil_msgs::AssignManipulatorTaskStep>::iterator step = presentWorkSeq->steps.begin(); step != presentWorkSeq->steps.end(); step++){

			/**
			 * Diagnostics
			 */
			diagnostic_msgs::DiagnosticStatus step_diag;

			step_diag.level = diagnostic_msgs::DiagnosticStatus::OK;	//errors are bad! MMmmmmk....
			step_diag.name = "WSM";										//Module name
			step_diag.message = "Doing a step";							//Short description
			step_diag.hardware_id = ""; 								//This is unique, so how to determine this hardware_id?

			//Send two information lines:
			step_diag.values.clear();
			PUSH_KEYVALUE(step_diag, "step_id", step->id);
			PUSH_KEYVALUE(step_diag, "status", "started");

			//Publish
			COMPONENT->publishDiagnostic(step_diag);
			/**
			 * End Diagnostics
			 */

			ros::Time stepTic = ros::Time::now();

			bool stepSuccess = true;
			bool stepTimeout = false;


			tf::StampedTransform sampledTF;
			config::WSM::pub::WSMVelocity twist;
			geometry_msgs::Pose t, initial_position;
			tf::Quaternion q, initq;



#ifndef WSM_USE_LOCALIZATION
			ros::NodeHandle n ;
			ros::ServiceClient gmscl=n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
			gazebo_msgs::GetModelState getmodelstate;
			getmodelstate.request.model_name ="Sahar";
#endif

			double rpy[3], initrpy[3];

			tf::StampedTransform body2loaderInit;
			tf::StampedTransform body2loaderCurrent;
			//config::WSM::sub::BladePosition bladePositionInit;
			//config::WSM::sub::BladePosition bladePositionCurrent;


			ROS_INFO("Got type %d", step->type);
			switch(step->type){
			case robil_msgs::AssignManipulatorTaskStep::type_unknown:


				break;
			case robil_msgs::AssignManipulatorTaskStep::type_blade_height:
				value = step->value; //height in meters

				if(step->blade_relativity == robil_msgs::AssignManipulatorTaskStep::blade_relativity_absolute){

				}else{

				}

				break;
			case robil_msgs::AssignManipulatorTaskStep::type_blade_angle:
				value = step->value; //angle in degrees (or rads??? conflict in documentation)

				body2loaderInit = COMPONENT->getLastTrasform("body", "loader");

				if(step->blade_relativity == robil_msgs::AssignManipulatorTaskStep::blade_relativity_absolute){

					double toc;
					double posdiff;
					config::WSM::pub::BladePositionCommand * bladeCommand;
					int sign = (value > 0) ? 1 : -1;
					do {
						//Set command to LLC
						bladeCommand = new config::WSM::pub::BladePositionCommand();
						bladeCommand-> velocity.push_back(0.3*sign);
						//bladeCommand->position.push_back(1);
						COMPONENT->publishBladePositionCommand(*bladeCommand);
						PAUSE(100);

						//Get difference
						body2loaderCurrent = COMPONENT->getLastTrasform("body", "loader");
						toc = (ros::Time::now() - stepTic).toSec();
						posdiff = body2loaderCurrent.getOrigin().z() - body2loaderInit.getOrigin().z();
						delete bladeCommand;
					}while(toc < step->success_timeout && ( fabs(value) - fabs(posdiff) > 0));
				}else{

				}


				break;
			case robil_msgs::AssignManipulatorTaskStep::type_clamp:
				value = step->value; //clamp in 0-100%


				break;
			case robil_msgs::AssignManipulatorTaskStep::type_advance:
				value = step->value; //advance in meters
				ROS_INFO("Got advance");

				if(step->blade_relativity == robil_msgs::AssignManipulatorTaskStep::blade_relativity_absolute){
#ifndef WSM_USE_LOCALIZATION
					gmscl.call(getmodelstate);
					initial_position = getmodelstate.response.pose;
#else
					initial_position = COMPONENT->receivedLocation->pose.pose;
#endif

					counter = 0;
					double toc;
					double posdiff;
					double sign = ((value > 0) ? 1 : -1);
					//Location control
					do{
						twist.twist.linear.x = 0.5 * sign; //Speed should be up to 1 meters per sec
						COMPONENT->publishWSMVelocity(twist);
						PAUSE(100);	//publish every 100ms
						//Get current position
#ifndef WSM_USE_LOCALIZATION
						gmscl.call(getmodelstate);
						t = getmodelstate.response.pose;
#else
						t = COMPONENT->receivedLocation->pose.pose;
#endif
						//Calculate time difference and location difference
						toc = (ros::Time::now() - stepTic).toSec();
						posdiff = t.position.x - initial_position.position.x;
						ROS_INFO("toc: %f", (float)toc);
						ROS_INFO("Traveled: %f", (float) posdiff);
					}
					while(toc < step->success_timeout && ( fabs(value) > fabs(posdiff)));
					ROS_INFO("FINISHED advance");

					twist.twist.linear.x = 0; //Set speed to 0
					//twist.header.frame_id = step->id;
					COMPONENT->publishWSMVelocity(twist);

				}else{

				}

				break;
			case robil_msgs::AssignManipulatorTaskStep::type_turn:
				value = step->value; //turn in degrees (or rads??? conflict in documentation)

				if(step->blade_relativity == robil_msgs::AssignManipulatorTaskStep::blade_relativity_graund /* <---WTF IS GRAUND?? */){

				}else{
#ifndef WSM_USE_LOCALIZATION
					gmscl.call(getmodelstate);
					initial_position = getmodelstate.response.pose;
#else
					initial_position = COMPONENT->receivedLocation->pose.pose;
#endif

					initq = tf::Quaternion(initial_position.orientation.x, initial_position.orientation.y, initial_position.orientation.z, initial_position.orientation.w);
					tf::Matrix3x3(initq).getRPY(initrpy[0], initrpy[1], initrpy[2]);

					//Location difference
					int sign = ((value > 0) ? 1 : -1);
					double yawdiff;
					do{
						twist.twist.angular.z = 0.5 * sign; //Speed is should be up to 0.3 radians per sec
						COMPONENT->publishWSMVelocity(twist);
						PAUSE(100);	//publish every 100ms
						//Get current position
#ifndef WSM_USE_LOCALIZATION
						gmscl.call(getmodelstate);
						t = getmodelstate.response.pose;
#else
						t = COMPONENT->receivedLocation->pose.pose;
#endif
						//Calculate algular changes in RPY
						q = tf::Quaternion(t.orientation.x, t.orientation.y, t.orientation.z, t.orientation.w);
						tf::Matrix3x3(q).getRPY(rpy[0], rpy[1], rpy[2]);

						yawdiff = rpy[2] - initrpy[2];
					}
					while(ros::Time::now() - stepTic < ros::Duration(step->success_timeout) && ( fabs(value) > fabs(yawdiff)));

					twist.twist.angular.z = 0; //Set speed to 0
					COMPONENT->publishWSMVelocity(twist);
				}

				break;

			}

			if(ros::Time::now() - stepTic > ros::Duration(step->success_timeout)){
				//Timeout
				stepTimeout = true;
				stepSuccess = false;
			}

			if(stepSuccess){
				PAUSE(step->duration_at_end);
			}



			/**
			 * Diagnostics
			 */
			step_diag.level = diagnostic_msgs::DiagnosticStatus::OK;	//errors are bad! MMmmmmk....
			step_diag.name = "WSM";										//Module name
			step_diag.message = "Doing a step";							//Short description
			step_diag.hardware_id = ""; 								//This is unique, so how to determine this hardware_id?

			//Send two information lines:
			step_diag.values.clear();
			PUSH_KEYVALUE(step_diag, "step_id", step->id);
			if(stepSuccess){
				PUSH_KEYVALUE(step_diag, "status", "Success");
			}else{
				if(stepTimeout) {
					PUSH_KEYVALUE(step_diag, "status", "Timeout");
				}else {
					PUSH_KEYVALUE(step_diag, "status", "Pause");
				}
			}

			//Publish
			COMPONENT->publishDiagnostic(step_diag);


			//End of step. Need to sleep here?
			//PAUSE(10000);
		}

		delete presentWorkSeq;
		presentWorkSeq = NULL;
		//End of task. Need to sleep here?
		//PAUSE(10000);
	}

	return TaskResult::SUCCESS();
}
TaskResult state_STANDBY(string id, const CallContext& context, EventQueue& events){
	//PAUSE(10000);
	ROS_INFO("WSM STANDBY");
	events.raiseEvent(Event("/wsm/Resume",context));

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

	ROS_INFO("Starting wsm (WorkSequnceManager)...");
	ROS_INFO("WSM AT FSM1");
	Fsmwsm(&context, &events);
	ROS_INFO("WSM AT FSM2");

}

geometry_msgs::Twist Translate(gazebo_msgs::GetModelState model_state){

/*std_msgs::Float64 x[3] ;
std_msgs::Float64 z[3] ;

std_msgs::Float64 a , b , c, d ;


a.data = model_state.response.pose.orientation.w ;
b.data = model_state.response.pose.orientation.x ;
c.data = model_state.response.pose.orientation.y ;
d.data = model_state.response.pose.orientation.z ;

x[0].data = (pow(a.data,2) + pow(b.data,2) - pow(c.data,2) - pow(d.data,2));
x[1].data = 2*b.data*c.data + 2*a.data*d.data ;
x[2].data = 2*b.data*d.data - 2*a.data*c.data ;

z[0].data = 2*b.data*d.data + 2*a.data*c.data ;
z[1].data = 2*c.data*d.data - 2*a.data*b.data ;
z[2].data = (pow(a.data,2)-pow(b.data,2) - pow(c.data,2)+pow(d.data,2));

for(int i = 0 ; i < 3 ; i++){

}*/

double sx = model_state.response.twist.linear.x ;
double sy = model_state.response.twist.linear.y ;
Quaternion qut(model_state.response.pose.orientation);
Rotation rot1 = GetRotation(qut);
double v = glSpeedloc(sx,sy,rot1.yaw);
geometry_msgs::Twist model_coordinates ;
model_coordinates.linear.x = v ;

return (model_coordinates);

}
