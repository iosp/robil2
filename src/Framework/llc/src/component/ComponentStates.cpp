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
	double v =glSpeedloc(sx,sy,rot1.yaw);
	geometry_msgs::Twist model_coordinates ;
	model_coordinates.linear.x = v ;

	return (model_coordinates);

}
TaskResult state_READY(string id, const CallContext& context, EventQueue& events){

	ROS_INFO("LLC Ready");

	double Kp = 0.3 , Kd = 0 , Ki = 5   ; 					/* PID constants of linear x */
	double Kpz = -0.5 , Kdz = 0  , Kiz = -0.5   ;					/* PID constants of angular z */
 	double dt = 0.001 ; 									/* control time interval */
	double integral [2] = {} ; 							/* integration part */
	double der [2] = {} ;  								/* the derivative of the error */
	int E_stop = 1; 									/* emergency stop */


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

		// Gazebo PID */
	    ros::ServiceClient gmscl=n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	    gazebo_msgs::GetModelState getmodelstate;
	    getmodelstate.request.model_name ="Sahar";
	    gmscl.call(getmodelstate);

	    t = Translate(getmodelstate);

	   	 //Gazebo PID//

	    cur_error.twist.linear.x =
	    	(COMPONENT->WPD_desired_speed.twist.linear.x - t.linear.x);
	    cur_error.twist.angular.z =
	    	(COMPONENT->WPD_desired_speed.twist.angular.z - getmodelstate.response.twist.angular.z);

	/* calculate integral and derivatives */
	integral[0] += ((cur_error.twist.linear.x )* dt);
	der[0] = ((cur_error.twist.linear.x - old_error.twist.linear.x)/dt);
	integral[1] += ((cur_error.twist.angular.z)* dt);
	der[1] = ((cur_error.twist.angular.z - old_error.twist.angular.z)/dt);

	/*Calculating the published data */
	/*
	Kp = COMPONENT->PID_CONSTANTS.twist.linear.x ;
	Kd = COMPONENT->PID_CONSTANTS.twist.linear.y ;
	Ki = COMPONENT->PID_CONSTANTS.twist.linear.z ;

	Kpz = COMPONENT->PID_CONSTANTS.twist.angular.x ;
	Kdz = COMPONENT->PID_CONSTANTS.twist.angular.y ;
	Kiz = COMPONENT->PID_CONSTANTS.twist.angular.z ;
*/

	Throttle_rate.data = (Kp*cur_error.twist.linear.x + Ki*integral[0] - Kd*der[0]) ;
	Steering_rate.data = E_stop*(Kpz*cur_error.twist.angular.z + Kiz*integral[1] - Kdz*der[1]) ;


	/* publish */

	COMPONENT->publishEffortsTh(Throttle_rate);
	COMPONENT->publishEffortsSt(Steering_rate);
/*
	cout << "========================= Linear x ===============================" << endl;
//	cout << "Reference : " <<  COMPONENT->WPD_desired_speed.twist.linear.x << endl ;
	cout << "Linear x speed:" << t.linear.x << endl;
//	cout << "measured speed:" << getmodelstate.response.twist.linear.x << endl ;
	cout << "The error for linear x:" <<  cur_error.twist.linear.x << endl;
	cout << "========================= Angular Z===================================" << endl;
	cout << "The error for angular z:" <<  cur_error.twist.angular.z << endl;
//	cout << "Reference : " <<  COMPONENT->WPD_desired_speed.twist.angular.z << endl ;
//	cout << "Angular z efforts published: " <<  Steering_rate.data  << endl ;
//	cout << "measured speed:" << getmodelstate.response.twist.angular.z << endl;
	cout << "======================================================================" << endl;
*/

	/* calibrate the error */
	old_error.twist.angular.z = cur_error.twist.angular.z ;
	old_error.twist.linear.x = cur_error.twist.linear.x ;

		if(!E_stop)
			break ;
		PAUSE(10);		/* wait dt time to recalculate error */

		//usleep(100000);
	}
	return TaskResult::SUCCESS();
}

TaskResult state_STANDBY(string id, const CallContext& context, EventQueue& events){
	//PAUSE(10000);
	
	Event e("Activation");
	events.raiseEvent(e);
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
