#include "ParameterTypes.h"
#include "InverseKinematics.h"
#include "helpermath.h"
#include "WsmTask.h"
#include "ComponentMain.h"
#include <gazebo_msgs/GetModelState.h>
#include <boost/thread.hpp>

#define WRAP_POSNEG_PI(x) atan2(sin(x), cos(x))
#define LIMIT(value, minim, maxim) std::max<double>(std::min<double>(value, maxim), minim)
#define COMPONENT_LINK this->_comp
//#define WSM_USE_LOCALIZATION

WsmTask::WsmTask(ComponentMain* comp)
{
	_taskid = 0 ;
	_cur_step = 0;
	_cur_WSD = NULL;
	_status = "NULL" ;
	_step_status = "NULL";
	_comp = comp;
}

WsmTask::WsmTask(int taskid , int cur_step ,const config::WSM::sub::WorkSeqData& cur_WSD , ComponentMain* comp)
	{
		_taskid = taskid ;
		_cur_step = 1 ;
		_cur_WSD = new config::WSM::sub::WorkSeqData (cur_WSD);
		_status = "active";
		_step_status = "active" ;
		_comp = comp;
	}

WsmTask::WsmTask(const WsmTask& other)
{
	this->_cur_WSD = other._cur_WSD ;
	this->_cur_step = other._cur_step ;
	this->_taskid = other._taskid ;
	this->_status = other._status ;
	this->_step_status = "active";
	this->_comp = other._comp;
}

WsmTask::~WsmTask()
{
	if(this->_cur_WSD  != NULL){
		delete this->_cur_WSD;
	}
}

void WsmTask::Save_state()
{
	if(this->_status == "active"){
		this->_step_status = "paused";
		this->_status = "paused";
		return ;
	}
}

void WsmTask::Load_state()
{
	this->_step_status = "active";
	this->_status = "active";
}

void WsmTask::push_key_value(diagnostic_msgs::DiagnosticStatus container , std::string key , std::string value)
{
	diagnostic_msgs::KeyValue kv;
	kv.key = key.c_str(); kv.value = value.c_str();
	container.values.push_back(kv);
	COMPONENT_LINK->publishDiagnostic(container);
}

void WsmTask::Set_Task_WSD(const config::WSM::sub::WorkSeqData &WSD)
{
	config::WSM::sub::WorkSeqData *g;
	g = new config::WSM::sub::WorkSeqData (WSD);
	this->_cur_WSD = g ;
	this->_status = "active";
	this->_taskid = atoi(this->_cur_WSD->task_id.c_str());
	this->_cur_step = 0 ;
	this->_step_status = "active";
}

void WsmTask::pauseTask()
{
	this->Set_task_status("paused");
	return;
}

void WsmTask::publish_step_diag(int before, int exit_status)
{
	std::string term_status ;
	diagnostic_msgs::DiagnosticStatus step_diag;
	diagnostic_msgs::KeyValue kv;

	step_diag.level = diagnostic_msgs::DiagnosticStatus::OK;
	step_diag.name = "WSM";
	step_diag.hardware_id = 1 ; /* Should ask what's this */

	if(before == 1)
	{
		step_diag.message = "Doing a step";
		step_diag.values.clear();
		WsmTask::push_key_value(step_diag , "step_id" ,boost::lexical_cast<std::string>(this->Get_WSD()->steps.front().id));
		WsmTask::push_key_value(step_diag , "status" , "started");
		//globalcomp->publish(step_diag);
		/*
		 * TODO:
		 * 		Get Gloabl Component here somehow
		 */
	//	ROS_INFO("Published pre-step diagnostics");
		//COMPONENT_LINK->publishDiagnostic(step_diag);
	}
	else
	{
		step_diag.message = "finished a step";
		step_diag.values.clear();
		WsmTask::push_key_value(step_diag , "step_id" , boost::lexical_cast<std::string>(this->Get_WSD()->steps.front().id));



		if(exit_status == 1){
			term_status = "Success";
		}
		else if(exit_status == -1){
			term_status = "paused";
		}
		else{
			term_status = "timed out";
		}

	//	ROS_INFO("[Exit status is:%d in execute publish]",exit_status);

		WsmTask::push_key_value(step_diag , "status", term_status);
		//globalcomp->publish(step_diag);
	//	ROS_INFO("Published post-step diagnostics");
		//COMPONENT_LINK->publishDiagnostic(step_diag);
		/*
		 * TODO:
		 * 		Publish after-diagnostics.
		 * 		% update the step status accordingly. (completed or timed out.)
		 */
	}

}

void WsmTask::monitor_time(double time)
{
	std_msgs::Header monitor ;
	std::string step_time = boost::lexical_cast<std::string>(time);
	monitor.frame_id = step_time.c_str();
	monitor.seq = this->Get_cur_step_index();
	monitor.stamp = ros::Time::now();
	COMPONENT_LINK->publish_monitor_time(monitor);
}

void WsmTask::execute_next_step()
{
	int exit_status = 0 ;

	switch(this->Get_step()->type)
	{
	case robil_msgs::AssignManipulatorTaskStep::type_unknown:
	//	ROS_INFO("Illegal Task type");
		break;
	case robil_msgs::AssignManipulatorTaskStep::type_blade_height:
		exit_status = this->handle_type_1();
		break;
	case robil_msgs::AssignManipulatorTaskStep::type_blade_angle:
		exit_status = this->handle_type_2();
		break;
	case robil_msgs::AssignManipulatorTaskStep::type_clamp:
		exit_status = this->handle_type_3();
		break;
	case robil_msgs::AssignManipulatorTaskStep::type_advance:
		exit_status = this->handle_type_4();
		break;
	case robil_msgs::AssignManipulatorTaskStep::type_turn:
		exit_status = this->handle_type_5();
		break;
	}
//	ROS_INFO("[Exit status is:%d in execute next step]",exit_status);

	this->publish_step_diag(0,exit_status);

	return ;
}

void WsmTask::Set_step_id(int index)
{
	this->_cur_step = index ;
}

void WsmTask::inter_step_sleep(unsigned int duration_at_end)
{
	sleep(duration_at_end);
	return;
}

void WsmTask::Set_task_status(string status)
{
	this->_status = status;
}

void WsmTask::Update_step()
{
	if(this->Get_status() == "paused" || this->Get_WSD()->steps.size() == 0 || this->Get_status() == "aborted"){
	//	ROS_INFO("Update step: mission got paused or aborted");
	//	ROS_INFO("Current step is still:%s",this->Get_step_id().c_str());
		return;
	}
	else{
		this->inter_step_sleep(this->Get_WSD()->steps.front().duration_at_end);
		//ROS_INFO("Erased step:%d",this->Get_WSD()->steps.front().id);
		this->Get_WSD()->steps.erase(this->Get_WSD()->steps.begin() , this->Get_WSD()->steps.begin()+ 1);
	}
	if(this->Get_WSD()->steps.size() >= 1){
	//	this->Set_step_id(Get_WSD()->steps.front().id + 1);
		this->Set_step_id(this->_cur_step + 1);
	//	ROS_INFO("forwarded next step: %d",this->Get_cur_step_index());
	}
	else{
		this->Set_task_status("complete");
	//	ROS_INFO("Mission in complete status now");
	}

	return;
}

robil_msgs::AssignManipulatorTaskStep* WsmTask::Get_step()
{
	return &(this->_cur_WSD->steps.front());
}

config::WSM::sub::WorkSeqData * WsmTask::Get_WSD()
{
	return this->_cur_WSD;
}

int WsmTask::Get_Task_id()
{
	return this->_taskid ;
}

int WsmTask::Get_cur_step_index()
{
	return this->_cur_step ;
}

string WsmTask::Get_status()
{
	return this->_status;
}

string WsmTask::Get_step_id()
{
	return boost::lexical_cast<std::string>(this->Get_WSD()->steps.front().id) ;
}

int WsmTask::handle_type_1()
{

robil_msgs::AssignManipulatorTaskStep* cur_step = &(this->Get_WSD()->steps.front());
const int N = 1000 ;

	ROS_INFO("Made type 1");
	double value = ((this->Get_WSD()->steps.front().value)+0.28748); 	//Height in meters from ground
	ROS_INFO("Value: %g",value);

		if(cur_step->blade_relativity == robil_msgs::AssignManipulatorTaskStep::blade_relativity_graund){
			value += COMPONENT_LINK->z_offset ;
		}

	double tic = ros::Time::now().toSec();			//Stores The current time
	tf::StampedTransform body2loaderCurrent;

			double toc ;
			double initialRPY [3] ;
			config::WSM::pub::BladePositionCommand * bladeCommand;
			int supporterStatesIndex = 0, loaderStatesIndex = 0;

			/* finds supporter and loader's indecies */
			for(int i = 0; i < COMPONENT_LINK->jointStates->name.size(); i++){
				if(COMPONENT_LINK->jointStates->name[i] == "supporter_joint"){
					supporterStatesIndex = i;
				}
				if(COMPONENT_LINK->jointStates->name[i] == "loader_joint"){
					loaderStatesIndex = i;
				}
			}

			/* David - INV KINEMATICS */

					double des_hight , des_pitch , H[N] , Q3[N] , loader[N] , dh;

					Q3[0] = COMPONENT_LINK->jointStates->position[supporterStatesIndex] ;
					loader[0] = COMPONENT_LINK->jointStates->position[loaderStatesIndex];
					des_hight = value ;
					body2loaderCurrent = COMPONENT_LINK->getLastTrasform("loader", "body");
					body2loaderCurrent.getBasis().getRPY(initialRPY[0],initialRPY[1],initialRPY[2],1);
					H[0] = body2loaderCurrent.getOrigin().z();
					des_pitch = initialRPY[1];
					dh = (value - H[0])/N ;
					double jacobi = 0;

					ROS_INFO("Initial values are: Q3 = %f , Loader = %f , Pitch = %f , Hight = %f DH = %f"  , Q3[0] , loader[0], des_pitch , H[0],dh);
					ROS_INFO("des_pitch: %f", des_pitch);
					//&&(this->Get_status()!="paused")
				/* Perform inverse Kinematics */
						for(int i = 1 ; (i < N)&&(this->Get_status()!="paused"); i++){
							jacobi = InverseKinematics::get_J(Q3[i-1]);
							Q3[i] = Q3[i-1] + (pow(jacobi,-1))*dh ;
							loader[i] = -(InverseKinematics::get_pitch(Q3[i],0)) + des_pitch ;
							bladeCommand = new config::WSM::pub::BladePositionCommand();

				/* publish to LLC every 5ms */
													bladeCommand->name.push_back("supporter_joint");
													bladeCommand->name.push_back("loader_joint");

													bladeCommand->position.push_back(Q3[i]);
													bladeCommand->position.push_back(loader[i]);

													COMPONENT_LINK->publishBladePositionCommand(*bladeCommand);
													delete bladeCommand;

							usleep(5000);
						toc = (ros::Time::now().toSec() - tic);
					    if(toc > cur_step->success_timeout)
						return 0;
				}
				/* Check for pause or success */
						/* Check for time out */
						if(this->Get_status() == "paused"){
							return -1 ;
						}
						else{
							this->monitor_time(toc);
							return 1 ;
						}
}

int WsmTask::handle_type_2()
{

	robil_msgs::AssignManipulatorTaskStep* cur_step = &(this->Get_WSD()->steps.front());
	const int N = 1000 ;
	double value = (this->Get_WSD()->steps.front().value);

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

			/* finds supporter and loader's indices */
			for(int i = 0; i < COMPONENT_LINK->jointStates->name.size(); i++){
				if(COMPONENT_LINK->jointStates->name[i] == "supporter_joint"){
					supporterStatesIndex = i;
				}
				if(COMPONENT_LINK->jointStates->name[i] == "loader_joint"){
					loaderStatesIndex = i;
				}
			}

			/* David -  inv_kinematics */
			double loader[N] ;
			body2loaderInit = COMPONENT_LINK->getLastTrasform("loader" , "body");
			body2loaderInit.getBasis().getRPY(type2rpy[0],type2rpy[1],type2rpy[2],1);
			ROS_INFO("The Calculated pitch is: %f",type2rpy[1]);
			double dpitch = (targetJointAngle - type2rpy[1])/N;
			loader[0] = COMPONENT_LINK->jointStates->position[loaderStatesIndex] ;

			for(int i = 1 ; (i < N)&&(this->Get_status() != "paused") ; i++){
					loader[i] = loader[i-1] + dpitch ;
					if(i%4 == 0){
						bladeCommand.name.push_back("loader_joint");
						bladeCommand.position.push_back(loader[i]);
						COMPONENT_LINK->publishBladePositionCommand(bladeCommand);
						usleep(10000);
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
		if(this->Get_status() != "paused")
			COMPONENT_LINK->publishBladePositionCommand(bladeCommand);
			/* Check for pause or success*/
		if(this->Get_status() == "paused"){
			return -1;
		}
		else if(toc > cur_step->success_timeout){
			return 0 ;
		}
		else{
			this->monitor_time(toc);
			return 1 ;
		}
	}
		return 0 ;
}

int WsmTask::handle_type_3()
{
	ROS_INFO("made type 3");

	robil_msgs::AssignManipulatorTaskStep* cur_step = &(this->Get_WSD()->steps.front());
	double value = cur_step->value; //clamp in 0-100%

	if(cur_step->blade_relativity == robil_msgs::AssignManipulatorTaskStep::blade_relativity_absolute){

		double tic = ros::Time::now().toSec() ;
		double targetJointAngle = LIMIT(value, 0, 100)/100.0;//(std::max<double>(std::min<double>(value, 100), 0)) / 100.0;

		double toc;
		double posdiff;
		config::WSM::pub::BladePositionCommand * bladeCommand;
		int sign = (value > 0) ? 1 : -1;
		int bracketStatesIndex = 0;

		for(int i = 0; i < COMPONENT_LINK->jointStates->name.size(); i++)
			if(COMPONENT_LINK->jointStates->name[i] == "brackets_joint"){
				bracketStatesIndex = i;
				break;
			}

		do {
			double currentSupportAngle = COMPONENT_LINK->jointStates->position[bracketStatesIndex];
			sign = (currentSupportAngle > targetJointAngle) ? -1 : 1;
			double speed = LIMIT(0.5 * fabs(currentSupportAngle-targetJointAngle), 0.005, 0.05) * sign;

			//Set command to LLC
			bladeCommand = new config::WSM::pub::BladePositionCommand();
			bladeCommand->name.push_back("brackets_joint");
			bladeCommand->position.push_back(LIMIT(currentSupportAngle + speed, 0, 1));
			COMPONENT_LINK->publishBladePositionCommand(*bladeCommand);
			usleep(10000);

			//Get difference
			currentSupportAngle = COMPONENT_LINK->jointStates->position[bracketStatesIndex];
			toc = (ros::Time::now().toSec() - tic);
			posdiff = currentSupportAngle - targetJointAngle;//body2loaderCurrent.getOrigin().z() - body2loaderInit.getOrigin().z();
			delete bladeCommand;

			//ROS_INFO("Angle diff: %f; sent speed: %f; toc: %f", posdiff, speed, toc);
		}while(toc < cur_step->success_timeout && ( fabs(posdiff) > 0.01) && (this->Get_status() != "paused"));

			if(this->Get_status() == "paused"){
				return -1;
			}
			else if(toc > cur_step->success_timeout){
				return 0 ;
			}
			else{
				this->monitor_time(toc);
				return 1 ;
			}
   	   }

	return 0 ;
}

int WsmTask::handle_type_4()
{
	ROS_INFO("made type 4");
	robil_msgs::AssignManipulatorTaskStep* cur_step = &(this->Get_WSD()->steps.front());

	double value = cur_step->value; //advance in meters
	config::WSM::pub::WSMVelocity twist;
	boost::thread *Thread_ptr ;

	if(cur_step->blade_relativity == robil_msgs::AssignManipulatorTaskStep::blade_relativity_graund){
		ROS_INFO("launched correction thread");
		Thread_ptr = new boost::thread(boost::bind(&WsmTask::blade_correction,this));
	}

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
			COMPONENT_LINK->publishWSMVelocity(twist);
			usleep(100000);
			//PAUSE(100);	//publish every 100ms

#ifndef WSM_USE_LOCALIZATION
			gmscl.call(getmodelstate);
			looptime = (ros::Time::now().toSec() - looptime) ;
			linear_speed = sqrt(pow(getmodelstate.response.twist.linear.x , 2) + pow(getmodelstate.response.twist.linear.y,2));
			travelled_x += linear_speed*(looptime);
#else
			looptime = (ros::Time::now().toSec() - looptime) ;
			linear_speed = sqrt(pow(COMPONENT_LINK->receivedPerVelocity->twist.linear.x , 2) + pow(COMPONENT_LINK->receivedPerVelocity->twist.linear.y,2));
			travelled_x += (linear_speed*(looptime));
#endif
			toc = (ros::Time::now().toSec() - tic);
		}
		while(toc < cur_step->success_timeout && fabs(travelled_x) < fabs(value) && (this->Get_status() != "paused"));
		ROS_INFO("advanced %g meters.",travelled_x);
		twist.twist.linear.x = 0; //Set speed to 0
		//twist.header.frame_id = step->id;
		COMPONENT_LINK->publishWSMVelocity(twist);

		if(cur_step->blade_relativity == robil_msgs::AssignManipulatorTaskStep::blade_relativity_graund){
			ROS_INFO(">>>> Joining with correction thread..");
			Thread_ptr->interrupt();
			Thread_ptr->join();
			delete Thread_ptr ;
		}

		if(this->Get_status() == "paused"){
			this->_cur_WSD->steps.front().value = (value - travelled_x);
			return -1;
		}
		else if(toc > cur_step->success_timeout){
			return 0 ;
		}
		else{
			this->monitor_time(toc);
			return 1 ;
		}
	}

int WsmTask::handle_type_5()
{
	robil_msgs::AssignManipulatorTaskStep* cur_step = &(this->Get_WSD()->steps.front());
	double value = (cur_step->value)*(M_PI / 180.0);
		boost::thread *Thread_ptr ;

		if(cur_step->blade_relativity == robil_msgs::AssignManipulatorTaskStep::blade_relativity_graund){
			ROS_INFO("launched correction thread");
			Thread_ptr = new boost::thread(boost::bind(&WsmTask::blade_correction,this));
		}

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

			twist.twist.linear.x = 0.0;
			do{
				loop_time = ros::Time::now().toSec() ;
				sign = ((value > traveledYaw) ? 1 : -1);
				twist.twist.angular.z = LIMIT(0.5 * fabs(traveledYaw - value), 0.3, 0.6) * sign; //Speed is should be up to 0.3 radians per sec
				COMPONENT_LINK->publishWSMVelocity(twist);
				//ROS_INFO("@WSM: angular speed z: %f; linear speed x: %f", twist.twist.angular.z, twist.twist.linear.x);

				usleep(100000);
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
			while((toc < cur_step->success_timeout) && ( fabs(traveledYaw - value) > 0.01)&&(this->Get_status() != "paused"));

			twist.twist.angular.z = 0; //Set speed to 0
			COMPONENT_LINK->publishWSMVelocity(twist);

			if(cur_step->blade_relativity == robil_msgs::AssignManipulatorTaskStep::blade_relativity_graund){
					ROS_INFO(">>>> Joining with correction thread..");
					Thread_ptr->interrupt();
					Thread_ptr->join();
					delete Thread_ptr ;
				}

			if(this->Get_status() == "paused"){
				this->_cur_WSD->steps.front().value = (value - traveledYaw)*(180.0 / M_PI) ;
				return -1;
			}
			else if(toc > cur_step->success_timeout){
				return 0 ;
			}
			else{
				this->monitor_time(toc);
				return 1 ;
			}

		return 0 ;
}

double WsmTask::find_Map_max (int x)
{
	double max = -100 ;

	for(int i = 12 ; i < 18 ; i++ )
	{
		if(COMPONENT_LINK->recivedMap->data[(x*30) + i].height > max)
		{
			max = COMPONENT_LINK->recivedMap->data[(x*30) + i].height;
		}
	}
	return max;
}

Vec3D WsmTask::deriveMapPixel (tf::StampedTransform blade2body)
{

	Vec3D map_pixel ;
	Vec3D xt_1(0,0,0) ; Vec3D xt_2(0,0,0); Vec3D xt_ms(0,0,0);
	tf::StampedTransform test ;
	InverseKinematics::RotMatrix R_WB ; InverseKinematics::RotMatrix R_BL ; InverseKinematics::RotMatrix R_WL ;
	double length = 0 ; int pixel_dist = 0;

	R_WB = InverseKinematics::getRotMatrix(COMPONENT_LINK->receivedLocation);

	test = COMPONENT_LINK->getLastTrasform("loader","body");

	geometry_msgs::PoseWithCovarianceStamped *q = new geometry_msgs::PoseWithCovarianceStamped ;

	q->pose.pose.orientation.w = test.getRotation().getW();
	q->pose.pose.orientation.x = test.getRotation().getX();
	q->pose.pose.orientation.y = test.getRotation().getY();
	q->pose.pose.orientation.z = test.getRotation().getZ();
	q->pose.pose.position.x = test.getOrigin().x();
	q->pose.pose.position.y = test.getOrigin().y();
	q->pose.pose.position.z = test.getOrigin().z();

	//ROS_INFO("Get rot? blade");

	R_BL = InverseKinematics::getRotMatrix(q);

	//ROS_INFO("mat mul");

	R_WL = InverseKinematics::Matrix_mul(R_WB , R_BL);

	//ROS_INFO("just some shit here");

	xt_1.x = R_WB.R[0][3];  xt_2.x = R_WL.R [0][3];
	xt_1.y = R_WB.R[1][3];  xt_2.y = R_WL.R [1][3];

	xt_ms.x = (xt_1.x - xt_2.x);
	xt_ms.y = (xt_1.y - xt_2.y);

	length = sqrt (pow(xt_ms.x , 2) + pow(xt_ms.y , 2));
	pixel_dist = static_cast<int>((ceil(length*5)));

	ROS_INFO("This is the distance:%d",pixel_dist);

	map_pixel.x = (50 - pixel_dist);
	map_pixel.y =  15 ;
	map_pixel.z = R_WL.R[2][3];
//	ROS_INFO("[Height by Calculation is: %g m]",R_WL.R[2][3]);
	//ROS_INFO("[Blade location in bobcat coordinates: (%g ,%g ,%g)",R_WL.R[0][3],R_WL.R[1][3],R_WL.R[2][3]);
	//ROS_INFO("Relevant pixel is: (%g , %g , %g)",map_pixel.x,map_pixel.y ,map_pixel.z);

	delete q ;
	return map_pixel ;
}

 void WsmTask::blade_correction()
{
	bool loop_on = true;
	double t_hold = 0.05 ;
	double err = 0 ;
	double delta = 0;
	double RPY [3] ;
	double jac = 0 ;
	double g_max = 0;
	Vec3D Map_pixel(0,0,0) ;
	int supporterStatesIndex = 0, loaderStatesIndex = 0;

	tf::StampedTransform body2loader = COMPONENT_LINK->getLastTrasform("loader","body");
	Map_pixel = WsmTask::deriveMapPixel(body2loader);

	g_max = find_Map_max((int)Map_pixel.x);

		if(g_max == -100){
			COMPONENT_LINK->ground_heigth = 0 ;
		}
		else{
			COMPONENT_LINK->ground_heigth = g_max  ;
		}

	double set_point = Map_pixel.z + COMPONENT_LINK->ground_heigth ;
	double cur_h = 0 ;

	ROS_INFO("Initial set point:[%g m]" , set_point);
	ROS_INFO("TF calc. height: [%g m]", (Map_pixel.z + (COMPONENT_LINK->ground_heigth)));
	ROS_INFO("Initial ground height: [%g m]", g_max);

	/* finds supporter and loader's indices. */
		for(int i = 0; i < COMPONENT_LINK->jointStates->name.size(); i++){
			if(COMPONENT_LINK->jointStates->name[i] == "supporter_joint"){
				supporterStatesIndex = i;
			}
			if(COMPONENT_LINK->jointStates->name[i] == "loader_joint"){
				loaderStatesIndex = i;
			}
		}
		config::WSM::pub::BladePositionCommand * bladeCommand;
		double supporter_angle = 0 , loader_angle = 0 ;
		double next_supporter_angle = 0 , next_loader_angle = 0 ;

while(loop_on){
	try{
		/* Calc error signal */
		body2loader = COMPONENT_LINK->getLastTrasform("loader","body");
		//ROS_INFO("Blade position is:[%g , %g , %g]", body2loader.getOrigin().x(),body2loader.getOrigin().y(), body2loader.getOrigin().z() + 0.28);

		Map_pixel = deriveMapPixel(body2loader);
		g_max = find_Map_max((int)Map_pixel.x) ;

		if(g_max == -100){
				COMPONENT_LINK->ground_heigth = 0 ;
			}
			else{
				COMPONENT_LINK->ground_heigth = g_max ;
			}

		cur_h = Map_pixel.z + COMPONENT_LINK->ground_heigth ;
		delta = (set_point - cur_h);

		ROS_INFO("===================================");
		ROS_INFO("current Height:[%g m]", cur_h );
		ROS_INFO("Initial set point:[%g m]" , set_point);
		ROS_INFO("Map says ground height is: [%g m]",g_max );
		ROS_INFO("Current delta: [%g]" , delta);
		ROS_INFO("===================================");

		/* Check if t_hold was crossed */

			if(fabs(delta) >= t_hold)
			{
		for(int i = 0 ; i < 2 ; i++){
			//	ROS_INFO("set point is:%g ,last:%g , delta is: %g",set_point,blade2ground,delta);
				supporter_angle = COMPONENT_LINK->jointStates->position[supporterStatesIndex];
				loader_angle = COMPONENT_LINK->jointStates->position[loaderStatesIndex];
				jac = InverseKinematics::get_J(supporter_angle);

				next_supporter_angle = supporter_angle + (pow(jac,-1))*(delta/2.0) ;
				next_loader_angle = -(InverseKinematics::get_pitch(next_supporter_angle,0)) + RPY[1];

			//	ROS_INFO("Next supporter angle:%g",next_supporter_angle);

				bladeCommand = new config::WSM::pub::BladePositionCommand();
				bladeCommand->name.push_back("supporter_joint");
				bladeCommand->name.push_back("loader_joint");

				bladeCommand->position.push_back(next_supporter_angle);
				bladeCommand->position.push_back(next_loader_angle);
				COMPONENT_LINK->publishBladePositionCommand(*bladeCommand);

				delete bladeCommand;
			}
		}
		/* sleep 1 sec */
		boost::this_thread::sleep(boost::posix_time::seconds(1));
	}
	catch(boost::thread_interrupted const&)
	{
		loop_on = false;
		ROS_INFO("Correction terminated");
	}
  }
}

/*
 Vec3D deriveMapPixel (tf::StampedTransform blade2body)
{

	Vec3D map_pixel ;
	Vec3D xt_1(0,0,0) ; Vec3D xt_2(0,0,0); Vec3D xt_ms(0,0,0);
	tf::StampedTransform test ;
	InverseKinematics::RotMatrix R_WB ; InverseKinematics::RotMatrix R_BL ; InverseKinematics::RotMatrix R_WL ;
	double length = 0 ; int pixel_dist = 0;

	//ROS_INFO("Get rot? loc");

	R_WB = InverseKinematics::getRotMatrix(Global_comp->receivedLocation);

	test = Global_comp->getLastTrasform("loader","body");

	geometry_msgs::PoseWithCovarianceStamped *q = new geometry_msgs::PoseWithCovarianceStamped ;

	q->pose.pose.orientation.w = test.getRotation().getW();
	q->pose.pose.orientation.x = test.getRotation().getX();
	q->pose.pose.orientation.y = test.getRotation().getY();
	q->pose.pose.orientation.z = test.getRotation().getZ();
	q->pose.pose.position.x = test.getOrigin().x();
	q->pose.pose.position.y = test.getOrigin().y();
	q->pose.pose.position.z = test.getOrigin().z();

	//ROS_INFO("Get rot? blade");

	R_BL = InverseKinematics::getRotMatrix(q);

	//ROS_INFO("mat mul");

	R_WL = InverseKinematics::Matrix_mul(R_WB , R_BL);

	//ROS_INFO("just some shit here");

	xt_1.x = R_WB.R[0][3];  xt_2.x = R_WL.R [0][3];
	xt_1.y = R_WB.R[1][3];  xt_2.y = R_WL.R [1][3];

	xt_ms.x = (xt_1.x - xt_2.x);
	xt_ms.y = (xt_1.y - xt_2.y);

	length = sqrt (pow(xt_ms.x , 2) + pow(xt_ms.y , 2));
	pixel_dist = static_cast<int>((ceil(length*5)));

	map_pixel.x = (50 - pixel_dist);
	map_pixel.y =  15 ;
	map_pixel.z = R_WL.R[2][3];
//	ROS_INFO("[Height by Calculation is: %g m]",R_WL.R[2][3]);
	//ROS_INFO("[Blade location in bobcat coordinates: (%g ,%g ,%g)",R_WL.R[0][3],R_WL.R[1][3],R_WL.R[2][3]);
	//ROS_INFO("Relevant pixel is: (%g , %g , %g)",map_pixel.x,map_pixel.y ,map_pixel.z);

	delete q ;
	return map_pixel ;

}
*
/*===============================================================
 * 		Handle Supporter manipulation (Type 1).
 * 		Param 1 : cur_step - step to be executed
 * 		return : 1 - if step was finished successful.
 * 				 0 - if time out occurred
 * 				-1 - if pause was received during execution.
 *================================================================

int handle_type_1(std::vector<robil_msgs::AssignManipulatorTaskStep>::iterator cur_step , const CallContext& context , EventQueue& events)
{
	double value = ((cur_step->value)+0.28748); 	//Height in meters from ground

		if(cur_step->blade_relativity == robil_msgs::AssignManipulatorTaskStep::blade_relativity_graund){
			value += blade2ground ;
		}

	double tic = ros::Time::now().toSec();			//Stores The current time
	tf::StampedTransform body2loaderCurrent;

			double toc ;
			double initialRPY [3] ;
			config::WSM::pub::BladePositionCommand * bladeCommand;
			int supporterStatesIndex = 0, loaderStatesIndex = 0;

			/* finds supporter and loader's indecies.
			for(int i = 0; i < jointStates.name.size(); i++){
				if(jointStates.name[i] == "supporter_joint"){
					supporterStatesIndex = i;
				}
				if(jointStates.name[i] == "loader_joint"){
					loaderStatesIndex = i;
				}
			}

			/* David - INV KINEMATICS

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

				/* Perform inverse Kinematics
						for(int i = 1 ; (i < N)&&(!pause_time) ; i++){
							jacobi = InverseKinematics::get_J(Q3[i-1]);
							Q3[i] = Q3[i-1] + (pow(jacobi,-1))*dh ;
							loader[i] = -(InverseKinematics::get_pitch(Q3[i],0)) + des_pitch ;
							bladeCommand = new config::WSM::pub::BladePositionCommand();

				/* publish to LLC every 5ms
													bladeCommand->name.push_back("supporter_joint");
													bladeCommand->name.push_back("loader_joint");

													bladeCommand->position.push_back(Q3[i]);
													bladeCommand->position.push_back(loader[i]);

													COMPONENT->publishBladePositionCommand(*bladeCommand);
													delete bladeCommand;
													PAUSE(5);
				/* Check for time out
				toc = (ros::Time::now().toSec() - tic);
				if(toc > cur_step->success_timeout)
				return 0;
				}
				/* Check for pause or success
						if(pause_time){
							return -1 ;
						}
						else{
							monitor_time(toc);
							return 1 ;
						}

}
*/
/*===============================================================
 * 		Handle Loader manipulation (Type 2).
 * 		Param 1 : cur_step - step to be executed
 * 		return : 1 - if step was finished successful.
 * 				 0 - if time out occurred
 * 				-1 - if pause was received during execution.
 *================================================================

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

			/* finds supporter and loader's indecies.
			for(int i = 0; i < jointStates.name.size(); i++){
				if(jointStates.name[i] == "supporter_joint"){
					supporterStatesIndex = i;
				}
				if(jointStates.name[i] == "loader_joint"){
					loaderStatesIndex = i;
				}
			}

			/* David -  inv_kinematics
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
			/* Check for pause or success
				if(pause_time){
					return -1 ;
				}
				else{
					monitor_time(toc);
					return 1 ;
				}
		}
		else
		{
			/* TODO :
			 * 	ground relativity blade manipulation..

		}
		return 0 ;
}
*/
/*===============================================================
 * 		Handle Clamp manipulation (Type 3).
 * 		Param 1 : cur_step - step to be executed
 * 		return : 1 - if step was finished successful.
 * 				 0 - if time out occurred
 * 				-1 - if pause was received during execution.
 *================================================================

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
				monitor_time(toc);
				return 1 ;
			}
   	   }
	else
	{
		/* TODO :
		 * 	ground relativity blade manipulation..

	}

	return 0 ;
}
*/
/*===============================================================
 * 		Handle Advance Task (Type 4).
 * 		Param 1 : cur_step - step to be executed
 * 		return : 1 - if step was finished successful.
 * 				 0 - if time out occurred
 * 				-1 - if pause was received during execution.
 *================================================================

int handle_type_4(std::vector<robil_msgs::AssignManipulatorTaskStep>::iterator cur_step , const CallContext& context , EventQueue& events)
{
	double value = cur_step->value; //advance in meters
	config::WSM::pub::WSMVelocity twist;
	boost::thread *Thread_ptr ;

	if(cur_step->blade_relativity == robil_msgs::AssignManipulatorTaskStep::blade_relativity_graund){
		ROS_INFO("launched correction thread");
		Thread_ptr = new boost::thread(blade_correction);
	}

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

		if(cur_step->blade_relativity == robil_msgs::AssignManipulatorTaskStep::blade_relativity_graund){
			ROS_INFO(">>>> Joining with correction thread..");
			Thread_ptr->interrupt();
			Thread_ptr->join();
			delete Thread_ptr ;
		}

		if(pause_time){
			cur_step->value = (value - travelled_x);
			return -1;
		}
		else if(toc > cur_step->success_timeout){
			return 0 ;
		}
		else{
			monitor_time(toc);
			return 1 ;
		}
	//}
	/*
	else{
		/*
		 * TODO:
		 * 		Ground relative manipulation \ keep stedy heigth ?
		}


	return 0 ;
}

/*===============================================================
 * 		Handle Turn Task (Type 5).
 * 		Param 1 : cur_step - step to be executed
 * 		return : 1 - if step was finished successful.
 * 				 0 - if time out occurred
 * 				-1 - if pause was received during execution.
 *================================================================

int handle_type_5(std::vector<robil_msgs::AssignManipulatorTaskStep>::iterator cur_step , const CallContext& context , EventQueue& events)
{
	double value = (cur_step->value)*(M_PI / 180.0);
	boost::thread *Thread_ptr ;

	if(cur_step->blade_relativity == robil_msgs::AssignManipulatorTaskStep::blade_relativity_graund){
		ROS_INFO("launched correction thread");
		Thread_ptr = new boost::thread(blade_correction);
	}

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

		if(cur_step->blade_relativity == robil_msgs::AssignManipulatorTaskStep::blade_relativity_graund){
				ROS_INFO(">>>> Joining with correction thread..");
				Thread_ptr->interrupt();
				Thread_ptr->join();
				delete Thread_ptr ;
			}

		if(pause_time){
			cur_step->value = (value - traveledYaw)*(180.0 / M_PI) ;
			return -1;
		}
		else if(toc > cur_step->success_timeout){
			return 0 ;
		}
		else{
			monitor_time(toc);
			return 1 ;
		}

	return 0 ;
}
*/
