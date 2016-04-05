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

using namespace std;
using namespace decision_making;
#include "ComponentStates.h"
double k_emrg=1;
double hb_time = 0 ;
const double t_out = 20.0 ;
static double Kp = 1.3 , Kd = 0.0 , Ki = 0.0   ; 				/* PID constants of linear x */
static double Kpz = -1.8 , Kdz = 0.01 , Kiz = -0.1   ;		/* PID constants of angular z */
bool debug_flag=false;
ros::Time com_check_time;
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

void llc_status_callback (const std_msgs::Float64 &msg)
{
	k_emrg=msg.data;
}


geometry_msgs::Twist Translate(geometry_msgs::PoseWithCovarianceStamped model_state , geometry_msgs::Twist model_speed){

	std_msgs::Float64 x[3] ;
	std_msgs::Float64 z[3] ;
	std_msgs::Float64 v[3] ;
	std_msgs::Float64 w[3] ;
	std_msgs::Float64 a , b , c, d ;

	geometry_msgs::Twist model_coordinates ;
	double model_coordinates_linear_speed ;
	double model_coordinates_angular_speed ;


	a.data = model_state.pose.pose.orientation.w ;
	b.data = model_state.pose.pose.orientation.x ;
	c.data = model_state.pose.pose.orientation.y ;
	d.data = model_state.pose.pose.orientation.z ;

	x[0].data = (pow(a.data,2) + pow(b.data,2) - pow(c.data,2) - pow(d.data,2));
	x[1].data = 2*b.data*c.data + 2*a.data*d.data ;
	x[2].data = 2*b.data*d.data - 2*a.data*c.data ;

	z[0].data = 2*b.data*d.data + 2*a.data*c.data ;
	z[1].data = 2*c.data*d.data - 2*a.data*b.data ;
	z[2].data = (pow(a.data,2)-pow(b.data,2) - pow(c.data,2)+pow(d.data,2));

	v[0].data = model_speed.linear.x;
	v[1].data = model_speed.linear.y;
	v[2].data = model_speed.linear.z;

	w[0].data = model_speed.angular.x;
	w[1].data = model_speed.angular.y;
	w[2].data = model_speed.angular.z;

	model_coordinates_linear_speed = (dot_prod(x,v,3) / norm(x,3));
	model_coordinates_angular_speed = (dot_prod(z , w , 3) / norm(z ,3));

	model_coordinates.linear.x = model_coordinates_linear_speed ;
	model_coordinates.angular.z = model_coordinates_angular_speed ;

	return (model_coordinates);

}

void dynamic_Reconfiguration_callback(llc::ControlParamsConfig &config, uint32_t level) {

	Kp=config.linearVelocity_P;
	Ki=config.linearVelocity_I;
	Kd=config.linearVelocity_D;
	Kpz=config.angularVelocity_P;
	Kiz=config.angularVelocity_I;
	Kdz=config.angularVelocity_D;
}

void comm_check_callback(const geometry_msgs::TwistStamped &msg)
{
com_check_time=ros::Time::now();
}

TaskResult state_READY(string id, const CallContext& context, EventQueue& events){



	ROS_INFO("LLC Ready");

	double integral [2] = {} ; 								/* integration part */
	double der [2] = {} ;  									/* the derivative of the error */
	double integral_limit [2] = {1,-0.3};					/* emergency stop */
	double angular_filter[1024] = {} ;
	double old_err = 0;
	int E_stop = 1;
	COMPONENT->t_flag = 0 ;



	geometry_msgs::Twist per_speed ;
	geometry_msgs::PoseWithCovarianceStamped per_location ;



	config::LLC::pub::EffortsSt Steering_rate ; 		/* steering rate  +- 1 */
	config::LLC::pub::EffortsTh Throttle_rate ;			/* Throttle rate  +- 1 */
	sensor_msgs::JointState Blade_pos;

	geometry_msgs::TwistStamped cur_error ; 			/* stores the current error signal */
	geometry_msgs::TwistStamped old_error ; 			/* stores the last error signal */
	geometry_msgs::Twist t ;							/* used for co-ordinates transform */
	ros::Subscriber link_to_platform ;
	ros::Subscriber comm_check_sub;
	ros::Publisher linear_error_publisher;
	ros::Publisher angular_error_publisher;
	ros::Publisher debug_publisher;
	ros::NodeHandle n;
	link_to_platform = n.subscribe("/Sahar/link_with_platform" , 100, hb_callback);
	comm_check_sub = n.subscribe("/WPD/Speed", 100,comm_check_callback);
	
	
	if(debug_flag==true)
		{
		linear_error_publisher = n.advertise<std_msgs::Float64>("/LLC/DEBUG/linear_error", 100);
		angular_error_publisher = n.advertise<std_msgs::Float64>("/LLC/DEBUG/angular_error", 100);
		debug_publisher = n.advertise<std_msgs::Float64>("/LLC/DEBUG/or_debug", 100);
		}
	COMPONENT->WPD_desired_speed.twist.linear.x = 0;
	COMPONENT->WPD_desired_speed.twist.angular.z = 0;
	COMPONENT->WSM_desired_speed.twist.linear.x = 0;
	COMPONENT->WSM_desired_speed.twist.angular.z = 0;

	/* PID loop */

	while (!hb_time);

		ROS_INFO("LLC Connected with platform");
		/*
		 * TODO: Diagnostics about connection
		 */

/*
 *  PID LOOP
 */

	while((ros::Time::now().toSec() - hb_time) < t_out){
		if(events.isTerminated() || !ros::ok()){			/* checks whether the line is empty, or node failed */
			ROS_INFO("STOPPED");
			return TaskResult::TERMINATED();
		}

	/* get measurements and calculate error signal */

		



	    per_location = COMPONENT->Per_pose ;
	    per_speed = COMPONENT->Per_measured_speed;
	    t = Translate(per_location, per_speed);
	
		double current_time_double =ros::Time::now().toSec();
		double com_check_time_double =com_check_time.toSec();
		if(current_time_double > com_check_time_double +0.05) k_emrg=0;
		else k_emrg=1;

		if(COMPONENT->WPD_desired_speed.twist.linear.x ||COMPONENT->WPD_desired_speed.twist.angular.z ){
				cur_error.twist.linear.x = (COMPONENT->WPD_desired_speed.twist.linear.x) - t.linear.x;
				cur_error.twist.angular.z = ((COMPONENT->WPD_desired_speed.twist.angular.z) - t.angular.z);
		}
		else{
				cur_error.twist.linear.x = (COMPONENT->WSM_desired_speed.twist.linear.x) - t.linear.x;
				cur_error.twist.angular.z = ((COMPONENT->WSM_desired_speed.twist.angular.z) - t.angular.z);
		}
	std_msgs::Float64 angular_error;
	std_msgs::Float64 linear_error;
	std_msgs::Float64 debug_f;
	angular_error.data=cur_error.twist.angular.z;
	linear_error.data=cur_error.twist.linear.x;
	debug_f.data=COMPONENT->WPD_desired_speed.twist.linear.x;
	if(debug_flag==true)
		{
		linear_error_publisher.publish(linear_error);
		angular_error_publisher.publish(angular_error);
		debug_publisher.publish(debug_f);
		}


			Push_elm(angular_filter,1024,cur_error.twist.angular.z);
			cur_error.twist.angular.z = avg_filter(angular_filter , 1024.0);
			cur_error.twist.linear.x = (1-0.125)*old_err + cur_error.twist.linear.x*(0.125);
			old_err = cur_error.twist.linear.x ;

			double dt = 0.001 ;

	/* calculate integral and derivatives */
	integral[0] += ((cur_error.twist.linear.x )* dt);
	der[0] = ((cur_error.twist.linear.x - old_error.twist.linear.x)/dt);
	integral[1] += ((cur_error.twist.angular.z)* dt);
	der[1] = ((cur_error.twist.angular.z - old_error.twist.angular.z)/dt);
	for(int k = 0 ; k < 2 ; k++){
		if(integral[k] > integral_limit[k] )
				integral[k] = integral_limit[k] ;
		else if (integral[k] < -integral_limit[k])
				integral[k] = -integral_limit[k] ;
	}

	
	 
	Throttle_rate.data =( Kp*cur_error.twist.linear.x + Ki*integral[0] + Kd*der[0] )*k_emrg;
	Steering_rate.data =-( Kpz*cur_error.twist.angular.z+Kdz*der[1]+Kiz*integral[1] )*k_emrg; 
	


	/* publish */
		COMPONENT->publishEffortsTh(Throttle_rate);
		COMPONENT->publishEffortsSt(Steering_rate);


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

		if(!E_stop)
			break ;
		PAUSE(10);		/* wait dt time to recalculate error */
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
	if (argc > 1)
    {
		if(strcmp(argv[1],"DEBUG")==0){
			debug_flag=true;
			ROS_INFO("DEBUG mode is ON");
		}
    }
	RosEventQueue events;
	CallContext context;
	context.createParameters(new Params(&component));
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
