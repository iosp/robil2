#include <iostream>
#include <ros/ros.h>
#include <decision_making/SynchCout.h>
#include <decision_making/BT.h>
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>
#include <gazebo_msgs/GetModelState.h>
#include "aux_functions.h"
#include <bondcpp/bond.h>
#include <llc/ControlParamsConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TwistStamped.h>
#include <robil_msgs/GpsSpeed.h>
using namespace std;
using namespace decision_making;
#include "ComponentStates.h"


double k_emrg=1;
double hb_time = 0 ;
const double t_out = 20.0 ;
static double Kp = 1.3 , Kd = 0.0 , Ki = 0.0   ; 				/* PID constants of linear x */
static double Kpz = -1.8 , Kdz = 0.01 , Kiz = -0.1   ;		/* PID constants of angular z */
ros::Time heartbeat_time;
ros::Time GPS_time;
ros::Time LOC_time;
double linear_vel,angular_vel;
double dt_GPS, dt_LOC;
double wanted_linear_vel, wanted_angular_vel;
	geometry_msgs::TwistStamped cur_error ; 			/* stores the current error signal */
	geometry_msgs::TwistStamped old_error ; 			/* stores the last error signal */
		double linear_integral;
	double angular_integral;
		double linear_der;
	double angular_der;
class Params: public CallContextParameters{
public:
	ComponentMain* comp;
	Params(ComponentMain* comp):comp(comp){}
	std::string str()const{return "";}
};
FSM(llc_ON)
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
				FSM_ON_EVENT("/llc/Standby", FSM_NEXT(STANDBY));
			}
		}
		FSM_STATE(STANDBY)
		{
			FSM_CALL_TASK(STANDBY)
			FSM_TRANSITIONS{
				FSM_ON_EVENT("/llc/Resume", FSM_NEXT(READY));
			}
		}

	}
	FSM_END
}

FSM(llc)
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
				FSM_ON_EVENT("/llc/Activation", FSM_NEXT(ON));
			}
		}
		FSM_STATE(ON)
		{
			FSM_CALL_FSM(llc_ON)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/Shutdown", FSM_NEXT(OFF));
				FSM_ON_EVENT("/llc/Shutdown", FSM_NEXT(OFF));
			}
		}

	}
	FSM_END
}

TaskResult state_OFF(string id, const CallContext& context, EventQueue& events){
	ROS_INFO("LLC OFF");
	//diagnostic_msgs::DiagnosticStatus status;
	//COMPONENT->publishDiagnostic(status);
	return TaskResult::SUCCESS();
}

TaskResult state_INIT(string id, const CallContext& context, EventQueue& events){
	//PAUSE(10000);
	ROS_INFO("LLC Init");
	Event e("EndOfInit");
	events.raiseEvent(e);
	return TaskResult::SUCCESS();
}

void hb_callback (const std_msgs::Bool &msg)
{
	if(msg.data == true )
		hb_time = ros::Time::now().toSec();
	if(msg.data == false)
		hb_time = t_out + 1.0;
}



void llc_status_heartbeat (const geometry_msgs::TwistStamped &msg)
{
	heartbeat_time = ros::Time::now();
	wanted_linear_vel=msg.twist.linear.x;
	wanted_angular_vel=msg.twist.angular.z;
}

void llc_status_callback (const std_msgs::Float64 &msg)
{
	k_emrg=msg.data;
}


void GPS_angular_vel_cb  (const robil_msgs::GpsSpeed &msg)
{
        
	linear_vel=msg.speed;
	dt_GPS=(ros::Time::now()-GPS_time).toSec();
	GPS_time=ros::Time::now();
	if(dt_GPS < 0.01)dt_GPS=0.01;
	linear_integral += ((cur_error.twist.linear.x )* dt_GPS);
	linear_der = ((cur_error.twist.linear.x - old_error.twist.linear.x)/dt_GPS);
	//ROS_INFO("dt_GPS  = %lf ", dt_GPS);
}


void LOC_linear_vel_cb  (const geometry_msgs::TwistStamped &msg)
{
        
	angular_vel=msg.twist.angular.z;
	dt_LOC=(ros::Time::now()-LOC_time).toSec();
	if(dt_LOC < 0.01) dt_LOC = 0.01;
	LOC_time=ros::Time::now();
	angular_integral += ((cur_error.twist.angular.z)* dt_LOC);
	angular_der = ((cur_error.twist.angular.z - old_error.twist.angular.z)/dt_LOC);
	//ROS_INFO("dt_LOC  = %lf ", dt_LOC);
}


void dynamic_Reconfiguration_callback(llc::ControlParamsConfig &config, uint32_t level) {

	Kp=config.linearVelocity_P;
	Ki=config.linearVelocity_I;
	Kd=config.linearVelocity_D;
	Kpz=config.angularVelocity_P;
	Kiz=config.angularVelocity_I;
	Kdz=config.angularVelocity_D;
}

TaskResult state_READY(string id, const CallContext& context, EventQueue& events){



	ROS_INFO("LLC Ready");

	double linear_der;
	double angular_der;
	double linear_integral_limit = 5;
	double angular_integral_limit = 5;
	double err_epsilon = 0.01;
	double old_err = 0;
	COMPONENT->t_flag = 0 ;




	config::LLC::pub::EffortsSt Steering_rate ; 		/* steering rate  +- 1 */
	config::LLC::pub::EffortsTh Throttle_rate ;			/* Throttle rate  +- 1 */
 	sensor_msgs::JointState Blade_pos;


	std_msgs::Float64 angular_error;
	std_msgs::Float64 linear_error;							/* used for co-ordinates transform */
	

	ros::NodeHandle n;

	ros::Subscriber link_to_heatbeat = n.subscribe("/WPD/Speed" , 100, llc_status_heartbeat);
	ros::Subscriber link_to_platform = n.subscribe("/Sahar/link_with_platform" , 100, hb_callback);
	ros::Subscriber GPS_angular_vel  = n.subscribe("/SENSORS/GPS/Speed" , 100, GPS_angular_vel_cb);
	ros::Subscriber LOC_linear_vel  = n.subscribe("/LOC/Velocity" , 100, LOC_linear_vel_cb);
	
	ros::Publisher linear_error_publisher = n.advertise<std_msgs::Float64>("linear_error", 100);
	ros::Publisher angular_error_publisher = n.advertise<std_msgs::Float64>("/angular_error", 100);
	ros::Publisher Throttle_rate_pub = n.advertise<std_msgs::Float64>("/LLC/EFFORTS/Throttle", 100);
	ros::Publisher Steering_rate_pub = n.advertise<std_msgs::Float64>("/LLC/EFFORTS/Steering", 100);
	

	/* PID loop */

	while (!hb_time);

		ROS_INFO("Connected with platform");
		/*
		 * TODO: Diagnostics about connection
		 */

/*
 *  PID LOOP
 */
        ros::Rate loop_rate(100);
	while((ros::Time::now().toSec() - hb_time) < t_out){
		if(events.isTerminated() || !ros::ok()){			/* checks whether the line is empty, or node failed */
			ROS_INFO("STOPPED");
			return TaskResult::TERMINATED();
		}

	/* get measurements and calculate error signal */




			if(wanted_linear_vel || wanted_angular_vel ){
				cur_error.twist.linear.x = (wanted_linear_vel) -linear_vel;
				cur_error.twist.angular.z = (wanted_angular_vel - angular_vel);
			}
			else{
				cur_error.twist.linear.x = (COMPONENT->WSM_desired_speed.twist.linear.x) - linear_vel;
				cur_error.twist.angular.z = ((COMPONENT->WSM_desired_speed.twist.angular.z) - angular_vel);
			}

	
	angular_error.data=cur_error.twist.angular.z;
	linear_error.data=cur_error.twist.linear.x;
	old_err = cur_error.twist.linear.x ;



	/* calculate integral and derivatives */
	//ROS_INFO("dt_GPS  = %lf   dt_LOC  = %lf ", dt_GPS,dt_LOC );

	angular_integral += ((cur_error.twist.angular.z)* dt_LOC);
	angular_der = ((cur_error.twist.angular.z - old_error.twist.angular.z)/dt_LOC);
	
	/* limits to the integral */
	if(linear_integral > linear_integral_limit )
			linear_integral = linear_integral_limit ;
	else if (linear_integral < -linear_integral_limit)
			linear_integral = -linear_integral_limit ;
		
	if(angular_integral > angular_integral_limit)
			angular_integral = angular_integral_limit ;
	else if (angular_integral < -angular_integral_limit)
			angular_integral = -angular_integral_limit ;
	
	/* reset the integral */
	if(abs(cur_error.twist.linear.x)<err_epsilon ) linear_integral=0;
	if(abs(cur_error.twist.angular.z)<err_epsilon ) angular_integral=0;

	if((ros::Time::now()-heartbeat_time).toSec()>0.1) k_emrg=0;
	else k_emrg=1;
	
	Throttle_rate.data =( Kp*cur_error.twist.linear.x + Ki*linear_integral + Kd*linear_der )*k_emrg;
	Steering_rate.data =( Kpz*cur_error.twist.angular.z+Kdz*angular_der+Kiz*angular_integral )*k_emrg; 
	


	/* publish */

	Throttle_rate_pub.publish(Throttle_rate);
	Steering_rate_pub.publish(Steering_rate);
	linear_error_publisher.publish(linear_error);
	angular_error_publisher.publish(angular_error);

	/* WSM blade controller */

	if(COMPONENT->t_flag){

		Blade_pos.name.clear();
		Blade_pos.name = COMPONENT->Blade_angle.name;
		Blade_pos.position.clear();
	for(int i = 0 ; i < COMPONENT->Blade_angle.position.size(); i++)
		Blade_pos.position.push_back(COMPONENT->Blade_angle.position[i]);
		COMPONENT->publishEffortsJn(Blade_pos);
		COMPONENT->t_flag = 0 ;
	}

	/* calibrate the error */
	old_error.twist.angular.z = cur_error.twist.angular.z ;
	old_error.twist.linear.x = cur_error.twist.linear.x ;
	loop_rate.sleep();
	}

	ROS_INFO("cannot connect with platform");
	/*
	 * TODO: break bond diagnostics
	 */
	// END PID loop
	return TaskResult::SUCCESS();
}

TaskResult state_STANDBY(string id, const CallContext& context, EventQueue& events){

	ROS_INFO("LLC Standby");

	return TaskResult::SUCCESS();
}

void runComponent(int argc, char** argv, ComponentMain& component){


	ros::init(argc, argv, "llc");
	ros_decision_making_init(argc, argv);
	RosEventQueue events;
	CallContext context;
	context.createParameters(new Params(&component));
	//events.async_spin();
	LocalTasks::registration("OFF",state_OFF);
	LocalTasks::registration("INIT",state_INIT);
	LocalTasks::registration("READY",state_READY);
	LocalTasks::registration("STANDBY",state_STANDBY);

    dynamic_reconfigure::Server<llc::ControlParamsConfig> server;
    dynamic_reconfigure::Server<llc::ControlParamsConfig>::CallbackType f;

    f = boost::bind(&dynamic_Reconfiguration_callback, _1, _2);
    server.setCallback(f);

	ROS_INFO("Starting llc...");
	Fsmllc(&context, &events);

}

