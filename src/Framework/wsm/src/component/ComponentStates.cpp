
#include <iostream>
#include <ros/ros.h>

#include <decision_making/SynchCout.h>
#include <decision_making/BT.h>
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>
#include <decision_making/DebugModeTracker.hpp>
#include <gazebo_msgs/GetModelState.h>


using namespace std;
using namespace decision_making;

#include "ComponentStates.h"

#define PUSH_KEYVALUE(oObject, kKey, vValue) {diagnostic_msgs::KeyValue kv; \
											kv.key = kKey; \
											kv.value = vValue; \
											oObject.values.push_back(kv); }

#define WRAP_POSNEG_PI(x) atan2(sin(x), cos(x))
#define LIMIT(value, minim, maxim) std::max<double>(std::min<double>(value, maxim), minim)
#define N 1000

bool pause_time = false ;
int last_index = 0 ;
bool new_seq = true ;
bool SensorConnection;
sensor_msgs::JointState jointStates;
config::WSM::sub::WorkSeqData * presentWorkSeq = NULL;


int handle_type_1(std::vector<robil_msgs::AssignManipulatorTaskStep>::iterator cur_step , const CallContext& context, EventQueue& events);
int handle_type_2(std::vector<robil_msgs::AssignManipulatorTaskStep>::iterator cur_step , const CallContext& context, EventQueue& events);
int handle_type_3(std::vector<robil_msgs::AssignManipulatorTaskStep>::iterator cur_step , const CallContext& context, EventQueue& events);
int handle_type_4(std::vector<robil_msgs::AssignManipulatorTaskStep>::iterator cur_step , const CallContext& context, EventQueue& events);
int handle_type_5(std::vector<robil_msgs::AssignManipulatorTaskStep>::iterator cur_step , const CallContext& context, EventQueue& events);

void JointStatesCallback(const sensor_msgs::JointStateConstPtr &msg)
{
	jointStates = sensor_msgs::JointState(*msg);
}

void pauseCallback(const std_msgs::StringConstPtr &msg)
{

	if(msg->data.find("PauseMission",0) != -1){
		pause_time = true ;
	}
	if(msg->data.find("ResumeTask",0) != -1){
		pause_time = false ;
	}
	else{
	//	ROS_INFO("It was nothing..");
		return;
	}
}

class Params: public CallContextParameters{
public:
	ComponentMain* comp;
	Params(ComponentMain* comp):comp(comp){}
	std::string str()const{return "";}
};


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

	//#define WSM_USE_LOCALIZATION

	ROS_INFO("WSM At Ready");

	ros::Time tic = ros::Time::now();
	ros::Time toc;
	config::WSM::sub::BladePosition * presentBladePos = NULL;

	bool mission_comp = true ;

	while(1){
		if(events.isTerminated() || !ros::ok()){			/* checks whether the line is empty, or node failed */
			ROS_INFO("STOPPED");
			return TaskResult::TERMINATED();
		}

		/* Check for new Task */
		if(COMPONENT->receivedWorkSeqData == NULL){
			PAUSE(1000);
			continue;
		}
		if(presentWorkSeq != NULL)
		ROS_INFO("Present Task id: %s",presentWorkSeq->task_id.c_str());

		if(presentWorkSeq != NULL){
			if(boost::lexical_cast<std::string>(presentWorkSeq->task_id) == boost::lexical_cast<std::string>(COMPONENT->receivedWorkSeqData->task_id)){
				new_seq = false ;
			}
		}
		else{
			presentWorkSeq = COMPONENT->receivedWorkSeqData ;
			COMPONENT->receivedWorkSeqData = NULL ;
			last_index = 0 ;
			new_seq = true;
		}

		/* Keep Task's step diagnostics */

	//	ROS_INFO("Got task: %s , Task id is: %s", presentWorkSeq->task_description.c_str(),presentWorkSeq->task_id.c_str());
		for(std::vector<robil_msgs::AssignManipulatorTaskStep>::iterator step = presentWorkSeq->steps.begin(); (step != presentWorkSeq->steps.end())&&(!pause_time); ++step){

	/* In case of a pause - continue back where you stopped */
		if(!new_seq){
			while(last_index != 0){
				step++;
				last_index--;
			}
		}
			/* Handle step diagnostics */

			diagnostic_msgs::DiagnosticStatus step_diag;

			step_diag.level = diagnostic_msgs::DiagnosticStatus::OK;	//errors are bad! MMmmmmkay?....
			step_diag.name = "WSM";										//Module name
			step_diag.message = "Doing a step";							//Short description
			step_diag.hardware_id = ""; 								//This is unique, so how to determine this hardware_id?

			//Send two information lines:
			step_diag.values.clear();
			PUSH_KEYVALUE(step_diag, "step_id", boost::lexical_cast<std::string>(step->id));
			PUSH_KEYVALUE(step_diag, "status", "started");

			//Publish
			COMPONENT->publishDiagnostic(step_diag);

			/* End step diagnostics */

		//	ROS_INFO("Got type %d", step->type);
		//	ROS_INFO("Doing step number %d",step->id);

			int exit_status = 0 ;

			switch(step->type){
	/* Type 0 : */
			case robil_msgs::AssignManipulatorTaskStep::type_unknown:

				break;

	/* Type 1 : */
			case robil_msgs::AssignManipulatorTaskStep::type_blade_height:

				exit_status = handle_type_1(step , context , events);
				break;

	/* Type 2 : */
			case robil_msgs::AssignManipulatorTaskStep::type_blade_angle:

				exit_status = handle_type_2(step , context , events);
				break;

	/* Type 3 : */
			case robil_msgs::AssignManipulatorTaskStep::type_clamp:

				exit_status = handle_type_3(step , context , events);
				break;

	/* Type 4 : */
			case robil_msgs::AssignManipulatorTaskStep::type_advance:

				exit_status = handle_type_4(step , context , events);
				break;

	/* Type 5 : */
			case robil_msgs::AssignManipulatorTaskStep::type_turn:
				exit_status = handle_type_5(step , context , events);
				break;
			}

 			/* Handle post-step diagnostics */

			step_diag.level = diagnostic_msgs::DiagnosticStatus::OK;	//errors are bad!
			step_diag.name = "WSM";										//Module name
			step_diag.message = "finished a step";						//Short description
			step_diag.hardware_id = ""; 								//This is unique, so how to determine this hardware_id?
			step_diag.values.clear();

			PUSH_KEYVALUE(step_diag, "step_id", boost::lexical_cast<std::string>(step->id));
			if(exit_status == 1){
				PUSH_KEYVALUE(step_diag, "status", "Success");
			}
			else{
				if(exit_status == 0) {
					PUSH_KEYVALUE(step_diag, "status", "Timeout");
					mission_comp = false ;
					ROS_INFO("timeout occured.");
				}else {
					PUSH_KEYVALUE(step_diag, "status", "Pause");
					mission_comp = false ;
					ROS_INFO("got paused.");
				}
			}

			//Publish
			COMPONENT->publishDiagnostic(step_diag);

			if(exit_status == 1){
				last_index ++ ;
				new_seq = true ;
				unsigned int inter_step_period = step->duration_at_end;
				sleep(inter_step_period);
			}
		}

			/* Completed mission */
		if(mission_comp){
			events.raiseEvent(Event("/CompleteTask"));
			ROS_INFO("Mission complete");
			last_index = 0;
			new_seq = true ;
			delete presentWorkSeq;
			presentWorkSeq = NULL;
			COMPONENT->receivedWorkSeqData = NULL ;
		}
	}
	return TaskResult::SUCCESS();
}

TaskResult state_STANDBY(string id, const CallContext& context, EventQueue& events){
	while(pause_time){
		PAUSE(1000);
	}
	events.raiseEvent(Event("/wsm/Resume",context));
	return TaskResult::SUCCESS();
}

void runComponent(int argc, char** argv, ComponentMain& component){

	ros::NodeHandle n;
	ros::Subscriber jointstatesSub = n.subscribe<sensor_msgs::JointState>("/Sahar/joint_states", 100, &JointStatesCallback);
	ros::Subscriber PauseMission = n.subscribe<std_msgs::String>("/decision_making/events" , 100 , &pauseCallback);

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

/*===============================================================
 * 		Handle Supporter manipulation (Type 1).
 * 		Param 1 : cur_step - step to be executed
 * 		return : 1 - if step was finished successful.
 * 				 0 - if time out occurred
 * 				-1 - if pause was received during execution.
 *================================================================
 */
int handle_type_1(std::vector<robil_msgs::AssignManipulatorTaskStep>::iterator cur_step , const CallContext& context , EventQueue& events)
{
	double value = ((cur_step->value)-0.28748); 	//Height in meters from ground
	double tic = ros::Time::now().toSec();			//Stors The current time
	tf::StampedTransform body2loaderCurrent;

		if(cur_step->blade_relativity == robil_msgs::AssignManipulatorTaskStep::blade_relativity_absolute){
			double toc ;
			double initialRPY [3] ;
			config::WSM::pub::BladePositionCommand * bladeCommand;
			int supporterStatesIndex = 0, loaderStatesIndex = 0;

			/* finds supporter and loader's indecies. */
			for(int i = 0; i < jointStates.name.size(); i++){
				if(jointStates.name[i] == "supporter_joint"){
					supporterStatesIndex = i;
				}
				if(jointStates.name[i] == "loader_joint"){
					loaderStatesIndex = i;
				}
			}

			/* David - INV KINEMATICS */

					double des_hight , des_pitch , H[N] , Q3[N] , loader[N] , dh;

					Q3[0] = jointStates.position[supporterStatesIndex] ;
					loader[0] = jointStates.position[loaderStatesIndex];
					des_hight = value ;
					body2loaderCurrent = COMPONENT->getLastTrasform("loader", "body");
					body2loaderCurrent.getBasis().getRPY(initialRPY[0],initialRPY[1],initialRPY[2],1);
					H[0] = body2loaderCurrent.getOrigin().z();
					des_pitch = initialRPY[1];
					dh = (value - H[0])/N ;
					double jacobi = 0;

					ROS_INFO("Initial values are: Q3 = %f , Loader = %f , Pitch = %f , Hight = %f DH = %f"  , Q3[0] , loader[0], des_pitch , H[0],dh);
					ROS_INFO("des_pitch: %f", des_pitch);

				/* Perform inverse Kinematics */
						for(int i = 1 ; (i < N)&&(!pause_time) ; i++){
							jacobi = InverseKinematics::get_J(Q3[i-1]);
							Q3[i] = Q3[i-1] + (pow(jacobi,-1))*dh ;
							loader[i] = -(InverseKinematics::get_pitch(Q3[i],0)) + des_pitch ;
							bladeCommand = new config::WSM::pub::BladePositionCommand();

				/* publish to LLC every 5ms */
													bladeCommand->name.push_back("supporter_joint");
													bladeCommand->name.push_back("loader_joint");

													bladeCommand->position.push_back(Q3[i]);
													bladeCommand->position.push_back(loader[i]);

													COMPONENT->publishBladePositionCommand(*bladeCommand);
													delete bladeCommand;
													PAUSE(5);
				/* Check for time out */
				toc = (ros::Time::now().toSec() - tic);
				if(toc > cur_step->success_timeout)
				return 0;
				}
				/* Check for pause or success */
						if(pause_time)
							return -1 ;
						else
							return 1 ;
		}
		else{
			/*TODO:
			 * 		ground relativity blade manipulation..
			 */
		}
		return 0 ;
}

/*===============================================================
 * 		Handle Loader manipulation (Type 2).
 * 		Param 1 : cur_step - step to be executed
 * 		return : 1 - if step was finished successful.
 * 				 0 - if time out occurred
 * 				-1 - if pause was received during execution.
 *================================================================
 */
int handle_type_2(std::vector<robil_msgs::AssignManipulatorTaskStep>::iterator cur_step , const CallContext& context , EventQueue& events)
{

	double value = cur_step->value; //angle in degrees

		if(cur_step->blade_relativity == robil_msgs::AssignManipulatorTaskStep::blade_relativity_absolute){

			//Parameter Angle can be between -30 degrees to 56 degrees
			//Joint varies between -0.554 and 0.985 radians correspondingly

			double tic = ros::Time::now().toSec() ;
			double targetJointAngle = ( M_PI_2 * (LIMIT(value, -30, 56)) / 90.0);
			double toc;
			double type2rpy[3];
			tf::StampedTransform body2loaderInit;
			config::WSM::pub::BladePositionCommand bladeCommand;
			int sign = (value > 0) ? 1 : -1;
			//ROS_INFO("Got angle: %f; is angle: %f", value, targetJointAngle);
			int loaderStatesIndex = 0;
			int supporterStatesIndex = 0 ;

			/* finds supporter and loader's indecies. */
			for(int i = 0; i < jointStates.name.size(); i++){
				if(jointStates.name[i] == "supporter_joint"){
					supporterStatesIndex = i;
				}
				if(jointStates.name[i] == "loader_joint"){
					loaderStatesIndex = i;
				}
			}

			/* David -  inv_kinematics */
			double loader[N] ;
			body2loaderInit = COMPONENT->getLastTrasform("loader" , "body");
			body2loaderInit.getBasis().getRPY(type2rpy[0],type2rpy[1],type2rpy[2],1);
			ROS_INFO("The Calculated pitch is: %f",type2rpy[1]);
			double dpitch = (targetJointAngle - type2rpy[1])/N;
			loader[0] = jointStates.position[loaderStatesIndex] ;

			for(int i = 1 ; (i < N)&&(!pause_time) ; i++){
					loader[i] = loader[i-1] + dpitch ;
					if(i%4 == 0){
						bladeCommand.name.push_back("loader_joint");
						bladeCommand.position.push_back(loader[i]);
						COMPONENT->publishBladePositionCommand(bladeCommand);
						PAUSE(10);
						bladeCommand.name.clear();
						bladeCommand.position.clear();
					}
					toc = (ros::Time::now().toSec() - tic);
						if(toc > cur_step->success_timeout)
						return 0;
				}
			bladeCommand.name.push_back("loader_joint");
			bladeCommand.position.push_back(loader[N-1]);
			ROS_INFO("current loader: %f",loader[N-1]);
		if(!pause_time)
			COMPONENT->publishBladePositionCommand(bladeCommand);
			/* Check for pause or success */
				if(pause_time)
					return -1 ;
				else
					return 1 ;
		}
		else
		{
			/* TODO :
			 * 	ground relativity blade manipulation..
			 */
		}
		return 0 ;
}

/*===============================================================
 * 		Handle Clamp manipulation (Type 3).
 * 		Param 1 : cur_step - step to be executed
 * 		return : 1 - if step was finished successful.
 * 				 0 - if time out occurred
 * 				-1 - if pause was received during execution.
 *================================================================
 */
int handle_type_3(std::vector<robil_msgs::AssignManipulatorTaskStep>::iterator cur_step , const CallContext& context , EventQueue& events)
{
	double value = cur_step->value; //clamp in 0-100%

	if(cur_step->blade_relativity == robil_msgs::AssignManipulatorTaskStep::blade_relativity_absolute){

		double tic = ros::Time::now().toSec() ;
		double targetJointAngle = LIMIT(value, 0, 100)/100.0;//(std::max<double>(std::min<double>(value, 100), 0)) / 100.0;

		double toc;
		double posdiff;
		config::WSM::pub::BladePositionCommand * bladeCommand;
		int sign = (value > 0) ? 1 : -1;
		int bracketStatesIndex = 0;

		for(int i = 0; i < jointStates.name.size(); i++)
			if(jointStates.name[i] == "brackets_joint"){
				bracketStatesIndex = i;
				break;
			}

		do {
			double currentSupportAngle = jointStates.position[bracketStatesIndex];
			sign = (currentSupportAngle > targetJointAngle) ? -1 : 1;
			double speed = LIMIT(0.5 * fabs(currentSupportAngle-targetJointAngle), 0.005, 0.05) * sign;

			//Set command to LLC
			bladeCommand = new config::WSM::pub::BladePositionCommand();
			bladeCommand->name.push_back("brackets_joint");
			bladeCommand->position.push_back(LIMIT(currentSupportAngle + speed, 0, 1));
			COMPONENT->publishBladePositionCommand(*bladeCommand);
			PAUSE(100);

			//Get difference
			currentSupportAngle = jointStates.position[bracketStatesIndex];
			toc = (ros::Time::now().toSec() - tic);
			posdiff = currentSupportAngle - targetJointAngle;//body2loaderCurrent.getOrigin().z() - body2loaderInit.getOrigin().z();
			delete bladeCommand;

			//ROS_INFO("Angle diff: %f; sent speed: %f; toc: %f", posdiff, speed, toc);
		}while(toc < cur_step->success_timeout && ( fabs(posdiff) > 0.01) && (!pause_time));

			if(pause_time){
				return -1;
			}
			else if(toc > cur_step->success_timeout){
				return 0 ;
			}
			else{
				return 1 ;
			}
   	   }
	else
	{
		/* TODO :
		 * 	ground relativity blade manipulation..
		 */
	}

	return 0 ;
}

/*===============================================================
 * 		Handle Advance Task (Type 4).
 * 		Param 1 : cur_step - step to be executed
 * 		return : 1 - if step was finished successful.
 * 				 0 - if time out occurred
 * 				-1 - if pause was received during execution.
 *================================================================
 */
int handle_type_4(std::vector<robil_msgs::AssignManipulatorTaskStep>::iterator cur_step , const CallContext& context , EventQueue& events)
{
	double value = cur_step->value; //advance in meters
	config::WSM::pub::WSMVelocity twist;

	if(cur_step->blade_relativity == robil_msgs::AssignManipulatorTaskStep::blade_relativity_absolute){

		double tic = ros::Time::now().toSec();

#ifndef WSM_USE_LOCALIZATION
		ros::NodeHandle n ;
		ros::ServiceClient gmscl=n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
		gazebo_msgs::GetModelState getmodelstate;
		getmodelstate.request.model_name ="Sahar";
		gmscl.call(getmodelstate);
#endif
		double toc;
		double travelled_x = 0;
		double linear_speed = 0 ;
		double sign = ((value > 0) ? 1 : -1);
		double looptime = 0 ;
		//Location integration
		do{
			looptime = ros::Time::now().toSec();
			twist.twist.linear.x = 0.5 * sign; //Speed should be up to 1 meters per sec
			COMPONENT->publishWSMVelocity(twist);
			PAUSE(100);	//publish every 100ms

#ifndef WSM_USE_LOCALIZATION
			gmscl.call(getmodelstate);
			looptime = (ros::Time::now().toSec() - looptime) ;
			linear_speed = sqrt(pow(getmodelstate.response.twist.linear.x , 2) + pow(getmodelstate.response.twist.linear.y,2));
			travelled_x += linear_speed*(looptime);
#else
			looptime = (ros::Time::now().toSec() - looptime) ;
			travelled_x += fabs(sqrt(pow(COMPONENT->receivedPerVelocity->twist.linear.x , 2) + pow(COMPONENT->receivedPerVelocity->twist.linear.y,2))) *(looptime);
#endif
			toc = (ros::Time::now().toSec() - tic);
		}
		while(toc < cur_step->success_timeout && fabs(travelled_x) < fabs(value) && (!pause_time));
		ROS_INFO("advanced %f meters.",travelled_x);
		twist.twist.linear.x = 0; //Set speed to 0
		//twist.header.frame_id = step->id;
		COMPONENT->publishWSMVelocity(twist);

		if(pause_time){
			cur_step->value = (value - travelled_x);
			return -1;
		}
		else if(toc > cur_step->success_timeout){
			return 0 ;
		}
		else{
			return 1 ;
		}
	}
	else{
		/*
		 * TODO:
		 * 		Ground relative manupulation
		 */

		}
}

/*===============================================================
 * 		Handle Turn Task (Type 5).
 * 		Param 1 : cur_step - step to be executed
 * 		return : 1 - if step was finished successful.
 * 				 0 - if time out occurred
 * 				-1 - if pause was received during execution.
 *================================================================
 */
int handle_type_5(std::vector<robil_msgs::AssignManipulatorTaskStep>::iterator cur_step , const CallContext& context , EventQueue& events)
{
	double value = (cur_step->value)*(M_PI / 180.0);

	if(cur_step->blade_relativity == robil_msgs::AssignManipulatorTaskStep::blade_relativity_graund){
		/*
		 * TODO:
		 * ground relativity
		 */
	}
	else{

		config::WSM::pub::WSMVelocity twist;
		geometry_msgs::TwistStamped prev_speed;
		geometry_msgs::TwistStamped current_speed;
		tf::Quaternion q, initq;
		geometry_msgs::Pose t, initial_position;
		double rpy[3], initrpy[3];

#ifndef WSM_USE_LOCALIZATION
		ros::NodeHandle n ;
		ros::ServiceClient gmscl=n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
		gazebo_msgs::GetModelState getmodelstate;
		getmodelstate.request.model_name ="Sahar";
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
		double toc ;
		double tic = ros::Time::now().toSec();
		double loop_time ;
		double traveledYaw = 0.0;
		int dt = 100; //ms
		twist.twist.linear.x = 0.0;
		do{
			loop_time = ros::Time::now().toSec() ;
			sign = ((value > traveledYaw) ? 1 : -1);
			twist.twist.angular.z = LIMIT(0.5 * fabs(traveledYaw - value), 0.3, 0.6) * sign; //Speed is should be up to 0.3 radians per sec
			COMPONENT->publishWSMVelocity(twist);
			//ROS_INFO("@WSM: angular speed z: %f; linear speed x: %f", twist.twist.angular.z, twist.twist.linear.x);
			PAUSE(dt);	//publish every 100ms
			//Get current position
#ifndef WSM_USE_LOCALIZATION
							gmscl.call(getmodelstate);
							t = getmodelstate.response.pose;
							prev_speed.twist = current_speed.twist;
							current_speed.twist = getmodelstate.response.twist;
							loop_time = (ros::Time::now().toSec() - loop_time);
							traveledYaw += current_speed.twist.angular.z * loop_time;
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
			toc = (ros::Time::now().toSec() - tic);
			//ROS_INFO("@WSM: Target: %f; current: %f; toc: %f", value, traveledYaw, toc);
			//yawdiff = rpy[2] - initrpy[2];
			//ROS_INFO("Contdition: %f > 0.01 ?", fabs(traveledYaw - value));
		}
		while((toc < cur_step->success_timeout) && ( fabs(traveledYaw - value) > 0.01)&&(!pause_time));

		twist.twist.angular.z = 0; //Set speed to 0
		COMPONENT->publishWSMVelocity(twist);
		if(pause_time){
			cur_step->value = (value - traveledYaw)*(180.0 / M_PI) ;
			return -1;
		}
		else if(toc > cur_step->success_timeout){
			return 0 ;
		}
		else{
			return 1 ;
		}
	}
	return 0 ;
}






