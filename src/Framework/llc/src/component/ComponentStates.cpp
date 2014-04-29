#include <iostream>
#include <ros/ros.h>
#include <decision_making/SynchCout.h>
#include <decision_making/BT.h>
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>
#include <gazebo_msgs/GetModelState.h>
using namespace std;
using namespace decision_making;
#include "ComponentStates.h"

class Params: public CallContextParameters{
public:
	ComponentMain* comp;
	Params(ComponentMain* comp):comp(comp){}
	std::string str()const{return "";}
};

FSM(LLC_ON)
{
	FSM_STATES
	{
		INIT,
		READY
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
			FSM_TRANSITIONS{}
		}

	}
	FSM_END
}

FSM(LLC)
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
			}
		}
		FSM_STATE(ON)
		{
			FSM_CALL_FSM(LLC_ON)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/Shutdown", FSM_NEXT(OFF));
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
	Event e("/EndOfInit");
	events.raiseEvent(e);
	return TaskResult::SUCCESS();
}
TaskResult state_READY(string id, const CallContext& context, EventQueue& events){

	ROS_INFO("LLC Ready");

	double Kp = 0.5 , Kd = 0.001 , Ki = 0 ; 					/* PID constants of linear x */
	double Kpz = 0.1, Kdz = 0.001 , Kiz = 0 ;				/* PID constants of angular z */
 	double dt = 0.01 ; 									/* control time interval */
	double integral [2] = {} ; 							/* integration part */
	double der [2] = {} ;  								/* the derivative of the error */
	int E_stop = 1; 									/* emergency stop */

	config::LLC::pub::EffortsSt Steering_rate ; 		/* steering rate percentage +- 100 % */
	config::LLC::pub::EffortsJn Joints_rate ; 			/* Joint rate percentage +- 100 % */
	config::LLC::pub::EffortsTh Throttle_rate ;			/* Throttle rate percentage +- 100 % */

	geometry_msgs::TwistStamped cur_error ; 			/* stores the current error signal */
	geometry_msgs::TwistStamped old_error ; 			/* stores the last error signal */

	ros::NodeHandle n ;

	/* set up dynamic reconfig */


	/* PID loop */
	while(1){
		if(events.isTerminated() || !ros::ok()){			/* checks whether the line is empty, or node failed */
			ROS_INFO("STOPPED");
			return TaskResult::TERMINATED();
		}
	/* get measurements and calculate error signal *
		ROS_INFO("curr error z: %f, per: %f", COMPONENT->WPD_desired_speed.twist.angular.z, COMPONENT->Per_measured_speed.twist.angular.z);
		ROS_INFO("curr error x: %f, per: %f", COMPONENT->WPD_desired_speed.twist.linear.x, COMPONENT->Per_measured_speed.twist.linear.x);
	*/
		// Gazebo PID */
		ros::ServiceClient gmscl=n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
		gazebo_msgs::GetModelState getmodelstate;
	    getmodelstate.request.model_name ="Sahar";
	    gmscl.call(getmodelstate);
	    /*
	    COMPONENT->WPD_desired_speed.twist.linear.x = getmodelstate.response.twist.linear.x;
	    COMPONENT->WPD_desired_speed.twist.angular.z = getmodelstate.response.twist.angular.z;
	    */
	    //Gazebo PID /


	    cur_error.twist.linear.x =
	    	(COMPONENT->WPD_desired_speed.twist.linear.x - getmodelstate.response.twist.linear.x );
	    cur_error.twist.angular.z =
	    	(COMPONENT->WPD_desired_speed.twist.angular.z - getmodelstate.response.twist.angular.z);

	/* calculate integral and derivatives */
	integral[0] += ((cur_error.twist.linear.x )* dt);
	der[0] = ((cur_error.twist.linear.x - old_error.twist.linear.x)/dt);
	integral[1] += ((cur_error.twist.angular.z)* dt);
	der[1] = ((cur_error.twist.angular.z - old_error.twist.angular.z)/dt);

	Throttle_rate.data = E_stop*(Kp*cur_error.twist.linear.x + Ki*integral[0] - Kd*der[0]) ;
	Steering_rate.data = E_stop*(Kpz*cur_error.twist.angular.z + Kiz*integral[1] - Kdz*der[1]) ;

	//ROS_INFO("Linear x error : %f" , cur_error.twist.linear.x);


	/* publish */
	COMPONENT->publishEffortsTh(Throttle_rate);
	COMPONENT->publishEffortsSt(Steering_rate);

	/* output data */
	/*
	if((Throttle_rate <= 100) && (Throttle_rate >= -100))
		COMPONENT->publishEffortsTh(Throttle_rate);
	if(Steering_rate <= 100 && Steering_rate >= -100)
		COMPONENT->publishEffortsSt(Steering_rate);
	*/
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

void runComponent(int argc, char** argv, ComponentMain& component){

	ros_decision_making_init(argc, argv);
	RosEventQueue events;
	CallContext context;
	context.createParameters(new Params(&component));
	//events.async_spin();
	LocalTasks::registration("OFF",state_OFF);
	LocalTasks::registration("INIT",state_INIT);
	LocalTasks::registration("READY",state_READY);

	ROS_INFO("Starting llc...");
	FsmLLC(&context, &events);

}







