
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

//#define WRAP_POSNEG_PI(phase) (phase > 0) ? (fmod(phase+M_PI, 2.0*M_PI)-M_PI) : -(fmod(-phase+M_PI, 2.0*M_PI)+M_PI);
#define WRAP_POSNEG_PI(x) atan2(sin(x), cos(x))
#define LIMIT(value, minim, maxim) std::max<double>(std::min<double>(value, maxim), minim)

sensor_msgs::JointState jointStates;
void JointStatesCallback(const sensor_msgs::JointStateConstPtr &msg)
{
	jointStates = sensor_msgs::JointState(*msg);
}

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
	//ROS_INFO("WSM OFF");
	return TaskResult::SUCCESS();
}
TaskResult state_INIT(string id, const CallContext& context, EventQueue& events){
	//PAUSE(10000);
	//ROS_INFO("WSM INIT");
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

	//ROS_INFO("WSM At Ready");

	while(1){
		if(events.isTerminated() || !ros::ok()){			/* checks whether the line is empty, or node failed */
			//ROS_INFO("STOPPED");
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

		//ROS_INFO("Got task %s", presentWorkSeq->task_description.c_str());
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


			//ROS_INFO("Got type %d", step->type);
			switch(step->type){
			case robil_msgs::AssignManipulatorTaskStep::type_unknown:


				break;
			case robil_msgs::AssignManipulatorTaskStep::type_blade_height:
				value = step->value; //height in meters


				body2loaderInit = COMPONENT->getLastTrasform("loader", "body");

				if(step->blade_relativity == robil_msgs::AssignManipulatorTaskStep::blade_relativity_absolute){


					//Parameter height can be between 0m and 1.5m
					//Joint varies between 0 and 1 radians correspondingly
					double targetJointAngle = LIMIT(value, 0, 1.5) * (2/3.0);


					double toc;
					double posdiff;
					config::WSM::pub::BladePositionCommand * bladeCommand;
					int sign = (value > 0) ? 1 : -1;

					int supporterStatesIndex = 0, loaderStatesIndex = 0;
					for(int i = 0; i < jointStates.name.size(); i++){
						if(jointStates.name[i] == "supporter_joint"){
							supporterStatesIndex = i;
						}
						if(jointStates.name[i] == "loader_joint"){
							loaderStatesIndex = i;
						}
					}




/*
					//ROS_INFO("Got height: %f; is angle: %f", value, targetJointAngle);

					double startHeight, endHeight, startSuppAngle, endSuppAngle, startLoadAngle, endLoadAngle;
					double rpy [3] ;
					startHeight = COMPONENT->getLastTrasform("loader", "body").getOrigin().z();
					endHeight = value;
					startSuppAngle = jointStates.position[supporterStatesIndex];
					endSuppAngle = startSuppAngle;
					startLoadAngle = jointStates.position[loaderStatesIndex];

					endLoadAngle = startLoadAngle;

					sign = (value > startHeight) ? 1 : -1;
					int N = 100;
					double dh = (value - startHeight)/N;
					// Grisha /\ox
					tf::Matrix3x3(COMPONENT->getLastTrasform("loader", "body").getRotation()).getRPY(rpy[0], rpy[1], rpy[2]);

				//	double refval = jointStates.position[loaderStatesIndex] ;

					float dh = (value - startHeight)/N;
					float J;
					for(int i=1;i<N;i++) {
						J = get_J(q3[i-1]);
						q3[i] = q3[i-1] + pow(J,-1) * dh;
					}
					float loader = -get_pitch(q3[N-1],0) +  desiredPitch;


					bladeCommand = new config::WSM::pub::BladePositionCommand();
					bladeCommand->name.push_back("supporter_joint");
					bladeCommand->name.push_back("loader_joint");
					bladeCommand->position.push_back(endSuppAngle);
					bladeCommand->position.push_back(endLoadAngle);
					COMPONENT->publishBladePositionCommand(*bladeCommand);
					PAUSE(2000);
					delete bladeCommand;

//					for(int i = 0; i < N; i++){
//						double F = InverseKinematics::SupporterInv(endSuppAngle);
//						endSuppAngle += dh/F; // q3
//
//						endLoadAngle = InverseKinematics::LoaderInv(endSuppAngle, rpy[1]);
//
//						//endHeight += 0.01 * sign;
//						//endSuppAngle += 0.01 * sign / InverseKinematics::SupporterInv(endSuppAngle);
//
//						ROS_INFO("@WSM: iteration %d: supp angle: start: %f; end: %f; F = %f", i, startSuppAngle, endSuppAngle, F);
//
//						bladeCommand = new config::WSM::pub::BladePositionCommand();
//						bladeCommand->name.push_back("supporter_joint");
//						bladeCommand->name.push_back("loader_joint");
//						bladeCommand->position.push_back(endSuppAngle);
//						bladeCommand->position.push_back(endLoadAngle);
//						COMPONENT->publishBladePositionCommand(*bladeCommand);
//						PAUSE((int)(1000*5.0/N));
//						delete bladeCommand;
//					}

					//endLoadAngle += 0.01 * sign / InverseKinematics::LoaderInv(endSuppAngle, );


					ROS_INFO("@WSM: supp angle: start: %f; end: %f", startSuppAngle, endSuppAngle);
*/

					//Set command to LLC



					double initialRPY[3], currentRPY[3];
					body2loaderInit.getBasis().getRPY(initialRPY[0], initialRPY[1], initialRPY[2]);

					ROS_INFO("Initial: height: %f; pitch: %f", body2loaderInit.getOrigin().z(), initialRPY[1]);
					ROS_INFO("Indexed: supp: %d; loader: %d", supporterStatesIndex, loaderStatesIndex);

					double prevQ3 = jointStates.position[supporterStatesIndex];
					double desiredpitch = initrpy[1];

					do {


						double dh = 0.01 * ((body2loaderCurrent.getOrigin().z() < value) ? 1 : -1);
						double j;
						j = InverseKinematics::get_J(prevQ3);
						double newq3 = prevQ3 + (1/j) * dh;
						double loader = -InverseKinematics::get_pitch(newq3, 0) + desiredpitch;

						double nextSupporterAngle = newq3;
						double nextLoaderAngle = loader;//InverseKinematics::get_pitch(newq3, loader);

						//ROS_INFO("@WSM: Next Supporter: %f; Next Loader: %f", nextSupporterAngle, nextLoaderAngle);

						prevQ3 = nextSupporterAngle;

//
//						body2loaderCurrent = COMPONENT->getLastTrasform("loader", "body");
//						body2loaderCurrent.getBasis().getRPY(currentRPY[0], currentRPY[1], currentRPY[2]);
//
//
//						ROS_INFO("1) Current: height: %f; pitch: %f", body2loaderCurrent.getOrigin().z(), currentRPY[1]);
//
//						int heightSign = (value - body2loaderCurrent.getOrigin().z() > 0) ? 1 : -1;
//						int pitchSign = (currentRPY[1] > initialRPY[1]) ? -1 : 1;
//
//						ROS_INFO("2) Signs: height: %d; pitch: %d", heightSign, pitchSign);
//
//						double supporterJointSpeed = 0.005 * heightSign;
//						double loaderJointSpeed = 0.0017 * heightSign;
//
//						ROS_INFO("3) Current angles: supp: %f; loader: %f", jointStates.position[supporterStatesIndex], jointStates.position[loaderStatesIndex]);
//						double nextSupporterAngle = jointStates.position[supporterStatesIndex] + supporterJointSpeed;
//						double nextLoaderAngle = jointStates.position[loaderStatesIndex] + loaderJointSpeed;




						//double nextSupporterAngle = jointStates.position[supporterStatesIndex] + 0.01 * sign / InverseKinematics::SupporterInv((double) jointStates.position[supporterStatesIndex]);

						//double nextLoaderAngle = jointStates.position[loaderStatesIndex] + 0.01 * sign / InverseKinematics::LoaderInv((double) jointStates.position[loaderStatesIndex]);

						//ROS_INFO("4) next angles: supp: %f; loader: %f", nextSupporterAngle, nextLoaderAngle);
						//double currentSupportAngle = body2loaderCurrent.getOrigin().z() * (2/3.0);



						//sign = (currentSupportAngle > targetJointAngle) ? -1 : 1;
						//double speed = LIMIT(1.0 * fabs(currentSupportAngle-targetJointAngle), 0.005, 0.05) * sign;//std::min<double>(0.05, 0.5 * fabs(currentSupportAngle-targetJointAngle)) * sign;


						//Set command to LLC
						bladeCommand = new config::WSM::pub::BladePositionCommand();
						//bladeCommand->name.push_back("back_arm_joint");
						//bladeCommand->name.push_back("main_arm_joint");
						bladeCommand->name.push_back("supporter_joint");
						bladeCommand->name.push_back("loader_joint");
						//bladeCommand->name.push_back("front_cylinder_joint");
//						bladeCommand->name.push_back("loader_joint");
//
//						brackets_joint


						bladeCommand->position.push_back(nextSupporterAngle);
						bladeCommand->position.push_back(nextLoaderAngle);
//						bladeCommand->position.push_back(0.0*sign);
//						bladeCommand->position.push_back(0.0*sign);
//						bladeCommand->position.push_back(0.0*sign);
						//bladeCommand->position.push_back(1);
						COMPONENT->publishBladePositionCommand(*bladeCommand);

						while(COMPONENT->getLastTrasform("loader", "body").stamp_ == body2loaderCurrent.stamp_){
							PAUSE(10);
						}
						//PAUSE(400);

						//Get difference
						body2loaderCurrent = COMPONENT->getLastTrasform("loader", "body");
						//currentSupportAngle = body2loaderCurrent.getOrigin().z() * (2/3.0);
						toc = (ros::Time::now() - stepTic).toSec();
						posdiff = value - body2loaderCurrent.getOrigin().z();//currentSupportAngle - targetJointAngle;//body2loaderCurrent.getOrigin().z() - body2loaderInit.getOrigin().z();
						delete bladeCommand;

						//ROS_INFO("Angle diff: %f; sent speed: %f; toc: %f", posdiff, speed, toc);
					}while(toc < step->success_timeout && ( fabs(posdiff) > 0.01));


				}else{

				}

				break;
			case robil_msgs::AssignManipulatorTaskStep::type_blade_angle:
				value = step->value; //angle in degrees (or rads??? conflict in documentation)


				if(step->blade_relativity == robil_msgs::AssignManipulatorTaskStep::blade_relativity_absolute){

					//Parameter height can be between 29.5 degrees to -60.5 degrees
					//Joint varies between 0 and 1.57 radians correspondingly
					double targetJointAngle = - M_PI_2 * (LIMIT(value, -60.5, 29.5) - 29.5) / 90.0;


					double toc;
					double posdiff;
					config::WSM::pub::BladePositionCommand * bladeCommand;
					int sign = (value > 0) ? 1 : -1;
					//ROS_INFO("Got angle: %f; is angle: %f", value, targetJointAngle);
					int loaderStatesIndex = 0;

					for(int i = 0; i < jointStates.name.size(); i++)
						if(jointStates.name[i] == "loader_joint"){
							loaderStatesIndex = i;
							break;
						}

					do {
						double currentSupportAngle = jointStates.position[loaderStatesIndex];
						sign = (currentSupportAngle > targetJointAngle) ? -1 : 1;
						double speed = -LIMIT(0.5 * fabs(currentSupportAngle-targetJointAngle), 0.002, 0.02) * sign;//std::min<double>(0.05, 0.5 * fabs(currentSupportAngle-targetJointAngle)) * sign;

						//Set command to LLC
						bladeCommand = new config::WSM::pub::BladePositionCommand();
						bladeCommand->name.push_back("loader_joint");
						bladeCommand->position.push_back(LIMIT(currentSupportAngle + speed, 0, M_PI_2));
						COMPONENT->publishBladePositionCommand(*bladeCommand);
						PAUSE(100);

						//Get difference
						currentSupportAngle = jointStates.position[loaderStatesIndex];
						toc = (ros::Time::now() - stepTic).toSec();
						posdiff = currentSupportAngle - targetJointAngle;//body2loaderCurrent.getOrigin().z() - body2loaderInit.getOrigin().z();
						delete bladeCommand;

						//ROS_INFO("Angle diff: %f; sent speed: %f; toc: %f", posdiff, speed, toc);
					}while(toc < step->success_timeout && ( fabs(posdiff) > 0.01));

				}else{

				}


				break;
			case robil_msgs::AssignManipulatorTaskStep::type_clamp:
				value = step->value; //clamp in 0-100%


				if(step->blade_relativity == robil_msgs::AssignManipulatorTaskStep::blade_relativity_absolute){

					//Parameter height can be between 29.5 degrees to -60.5 degrees
					//Joint varies between 0 and 1.57 radians correspondingly
					double targetJointAngle = LIMIT(value, 0, 100)/100.0;//(std::max<double>(std::min<double>(value, 100), 0)) / 100.0;


					double toc;
					double posdiff;
					config::WSM::pub::BladePositionCommand * bladeCommand;
					int sign = (value > 0) ? 1 : -1;
					//ROS_INFO("Got angle: %f; is angle: %f", value, targetJointAngle);
					int bracketStatesIndex = 0;

					for(int i = 0; i < jointStates.name.size(); i++)
						if(jointStates.name[i] == "brackets_joint"){
							bracketStatesIndex = i;
							break;
						}

					do {
						double currentSupportAngle = jointStates.position[bracketStatesIndex];
						sign = (currentSupportAngle > targetJointAngle) ? -1 : 1;
						double speed = LIMIT(0.5 * fabs(currentSupportAngle-targetJointAngle), 0.005, 0.05) * sign;//std::min<double>(0.05, 0.5 * fabs(currentSupportAngle-targetJointAngle)) * sign;

						//Set command to LLC
						bladeCommand = new config::WSM::pub::BladePositionCommand();
						bladeCommand->name.push_back("brackets_joint");
						bladeCommand->position.push_back(LIMIT(currentSupportAngle + speed, 0, 1));
						COMPONENT->publishBladePositionCommand(*bladeCommand);
						PAUSE(100);

						//Get difference
						currentSupportAngle = jointStates.position[bracketStatesIndex];
						toc = (ros::Time::now() - stepTic).toSec();
						posdiff = currentSupportAngle - targetJointAngle;//body2loaderCurrent.getOrigin().z() - body2loaderInit.getOrigin().z();
						delete bladeCommand;

						//ROS_INFO("Angle diff: %f; sent speed: %f; toc: %f", posdiff, speed, toc);
					}while(toc < step->success_timeout && ( fabs(posdiff) > 0.01));



				}else{

				}


				break;
			case robil_msgs::AssignManipulatorTaskStep::type_advance:
				value = step->value; //advance in meters
				//ROS_INFO("Got advance");

				if(step->blade_relativity == robil_msgs::AssignManipulatorTaskStep::blade_relativity_absolute){
					//ROS_INFO("WSM: Blade absolute");
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
						posdiff = sqrt(pow(t.position.x - initial_position.position.x, 2) + pow(t.position.y - initial_position.position.y, 2) + pow(t.position.z - initial_position.position.z, 2));
						//ROS_INFO("toc: %f", (float)toc);
						//ROS_INFO("Traveled: %f", (float) posdiff);
					}
					while(toc < step->success_timeout && ( fabs(value) > fabs(posdiff)));
					//ROS_INFO("FINISHED advance");

					twist.twist.linear.x = 0; //Set speed to 0
					//twist.header.frame_id = step->id;
					COMPONENT->publishWSMVelocity(twist);

				}else{
					//ROS_INFO("WSM: Blade relative");

				}

				break;
			case robil_msgs::AssignManipulatorTaskStep::type_turn:
				value = step->value; //turn in degrees (or rads??? conflict in documentation)

				if(step->blade_relativity == robil_msgs::AssignManipulatorTaskStep::blade_relativity_graund /* <---WTF IS GRAUND?? */){

				}else{


					geometry_msgs::TwistStamped prev_speed;
					geometry_msgs::TwistStamped current_speed;

#ifndef WSM_USE_LOCALIZATION
					gmscl.call(getmodelstate);
					initial_position = getmodelstate.response.pose;
					prev_speed.twist = getmodelstate.response.twist;
					current_speed.twist = getmodelstate.response.twist;
#else
					initial_position = COMPONENT->receivedLocation->pose.pose;

					prev_speed.twist = COMPONENT->receivedPerVelocity->twist;
					prev_speed.header = COMPONENT->receivedPerVelocity->header;

					current_speed.twist = COMPONENT->receivedPerVelocity->twist;
					current_speed.header = COMPONENT->receivedPerVelocity->header;
#endif
					initq = tf::Quaternion(initial_position.orientation.x, initial_position.orientation.y, initial_position.orientation.z, initial_position.orientation.w);
					tf::Matrix3x3(initq).getRPY(initrpy[0], initrpy[1], initrpy[2]);


					//Location difference
					double target_yaw = WRAP_POSNEG_PI(initrpy[2] + value);//fmod(initrpy[2] + value + M_PI, 2 * M_PI) - M_PI;
					int sign = ((value > 0) ? 1 : -1);
					double toc;
					double traveledYaw = 0.0;
					int dt = 100; //ms
					twist.twist.linear.x = 0.0;
					do{
						sign = ((value > traveledYaw) ? 1 : -1);
						twist.twist.angular.z = LIMIT(0.5 * fabs(traveledYaw - value), 0.2, 0.5) * sign; //Speed is should be up to 0.3 radians per sec
						COMPONENT->publishWSMVelocity(twist);
						//ROS_INFO("@WSM: angular speed z: %f; linear speed x: %f", twist.twist.angular.z, twist.twist.linear.x);
						PAUSE(dt);	//publish every 100ms
						//Get current position
#ifndef WSM_USE_LOCALIZATION
						gmscl.call(getmodelstate);
						t = getmodelstate.response.pose;
						prev_speed.twist = current_speed.twist;
						current_speed.twist = getmodelstate.response.twist;

						traveledYaw += current_speed.twist.angular.z * (dt / 1000.0);
#else
						t = COMPONENT->receivedLocation->pose.pose;
						prev_speed = current_speed;
						current_speed.twist = COMPONENT->receivedPerVelocity->twist;
						current_speed.header = COMPONENT->receivedPerVelocity->header;

						traveledYaw += current_speed.twist.angular.z * (current_speed.header.stamp - prev_speed.header.stamp).toSec();
#endif


						//Calculate angular changes in RPY
						q = tf::Quaternion(t.orientation.x, t.orientation.y, t.orientation.z, t.orientation.w);
						tf::Matrix3x3(q).getRPY(rpy[0], rpy[1], rpy[2]);



						toc = (ros::Time::now() - stepTic).toSec();
						//ROS_INFO("@WSM: Target: %f; current: %f; toc: %f", value, traveledYaw, toc);
						//yawdiff = rpy[2] - initrpy[2];
					}
					while(toc < step->success_timeout && ( fabs(traveledYaw - value) > 0.01));

					twist.twist.angular.z = 0; //Set speed to 0
					COMPONENT->publishWSMVelocity(twist);
					//ROS_INFO("@WSM: STOP blatt");
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
	//ROS_INFO("WSM STANDBY");
	events.raiseEvent(Event("/wsm/Resume",context));

	return TaskResult::SUCCESS();
}

void runComponent(int argc, char** argv, ComponentMain& component){

	ros::NodeHandle n;
	ros::Subscriber jointstatesSub = n.subscribe<sensor_msgs::JointState>("/Sahar/joint_states", 100, &JointStatesCallback);

	ros_decision_making_init(argc, argv);
	RosEventQueue events;
	CallContext context;
	context.createParameters(new Params(&component));
	//events.async_spin();
	LocalTasks::registration("OFF",state_OFF);
	LocalTasks::registration("INIT",state_INIT);
	LocalTasks::registration("READY",state_READY);
	LocalTasks::registration("STANDBY",state_STANDBY);

	//ROS_INFO("Starting wsm (WorkSequnceManager)...");
	//ROS_INFO("WSM AT FSM1");
	Fsmwsm(&context, &events);
	//ROS_INFO("WSM AT FSM2");

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
