#include <iostream>
#include <ros/ros.h>
#include <decision_making/SynchCout.h>
#include <decision_making/BT.h>
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>
#include <gazebo_msgs/GetModelState.h>
#include "aux_functions.h"
#include "helpermath.h"
using namespace std;
using namespace decision_making;
#include "ComponentStates.h"

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
	/*
	double sx = model_state.response.twist.linear.x ;
	double sy = model_state.response.twist.linear.y ;
	Quaternion qut(model_state.response.pose.orientation);
	Rotation rot1 = GetRotation(qut);
	double v =glSpeedloc(sx,sy,rot1.yaw);
	geometry_msgs::Twist model_coordinates ;
	model_coordinates.linear.x = v ;


	return (model_coordinates);
	*/

}
TaskResult state_READY(string id, const CallContext& context, EventQueue& events){

	ROS_INFO("LLC Ready");

	double Kp = 0.3 , Kd = 0 , Ki = 0   ; 				/* PID constants of linear x */
	double Kpz = -0.5 , Kdz = 0  , Kiz = 0   ;			/* PID constants of angular z */
 	double dt = 0.001 ; 								/* control time interval */
	double integral [2] = {} ; 							/* integration part */
	double der [2] = {} ;  								/* the derivative of the error */
	double integral_limit [2] = {0.1,0.05};
	int E_stop = 1; 									/* emergency stop */
	double angular_filter[5];
	COMPONENT->t_flag = 0 ;

	config::LLC::pub::EffortsSt Steering_rate ; 		/* steering rate percentage +- 100 % */
	config::LLC::pub::EffortsJn Joints_rate ; 			/* Joint rate percentage +- 100 % */
	config::LLC::pub::EffortsTh Throttle_rate ;			/* Throttle rate percentage +- 100 % */

	geometry_msgs::TwistStamped cur_error ; 			/* stores the current error signal */
	geometry_msgs::TwistStamped old_error ; 			/* stores the last error signal */
	geometry_msgs::Twist t ;

	ros::NodeHandle n ;

	/* set up dynamic reconfig */


	/* PID loop */
	while(1){
		if(events.isTerminated() || !ros::ok()){			/* checks whether the line is empty, or node failed */
			ROS_INFO("STOPPED");
			return TaskResult::TERMINATED();
		}

	/* get measurements and calculate error signal */

/*
	    ros::ServiceClient gmscl=n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	    gazebo_msgs::GetModelState getmodelstate;
	    getmodelstate.request.model_name ="Sahar";
	    gmscl.call(getmodelstate);
	    	t = Translate(getmodelstate);

	    	/* printing */

			t = Translate(COMPONENT->Per_pose , COMPONENT->Per_measured_speed.twist) ;

			if(COMPONENT->WPD_desired_speed.twist.linear.x ||COMPONENT->WPD_desired_speed.twist.angular.z ){
				cur_error.twist.linear.x = (COMPONENT->WPD_desired_speed.twist.linear.x) - t.linear.x;
				cur_error.twist.angular.z = ((COMPONENT->WPD_desired_speed.twist.angular.z) - t.linear.z);
			}
			else{
				cur_error.twist.linear.x = (COMPONENT->WSM_desired_speed.twist.linear.x) - t.linear.x;
				cur_error.twist.angular.z = ((COMPONENT->WSM_desired_speed.twist.angular.z) - t.linear.z);
			}

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

	Throttle_rate.data = (Kp*cur_error.twist.linear.x + Ki*integral[0] - Kd*der[0]) ;
	Steering_rate.data = E_stop*(Kpz*cur_error.twist.angular.z + Kiz*integral[1] - Kdz*der[1]) ;
	if(COMPONENT->t_flag)
		Joints_rate.data = COMPONENT->Blade_angle.position.front();
	else
		Joints_rate.data = 0;


	/* publish */

	COMPONENT->publishEffortsTh(Throttle_rate);
	COMPONENT->publishEffortsSt(Steering_rate);
	COMPONENT->publishEffortsJn(Joints_rate);

	/* calibrate the error */
	old_error.twist.angular.z = cur_error.twist.angular.z ;
	old_error.twist.linear.x = cur_error.twist.linear.x ;

		if(!E_stop)
			break ;
		PAUSE(100);		/* wait dt time to recalculate error */

		//usleep(100000);
	}
	return TaskResult::SUCCESS();
}

TaskResult state_STANDBY(string id, const CallContext& context, EventQueue& events){
	//PAUSE(10000);
	//Event e("Activation");
	//events.raiseEvent(e);
	ROS_INFO("LLC Standby");

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

	ROS_INFO("Starting llc...");
	Fsmllc(&context, &events);

}
