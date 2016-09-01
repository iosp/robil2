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
#include "ComponentStates.h"
#include <math.h>

using namespace std;
using namespace decision_making;
using namespace ros;

#define DT 0.01
#define LENGTH_OF_RECORD_IN_SECONDS 10
#define LENGTH_OF_RECORD_IN_FRAMES (1/DT)*LENGTH_OF_RECORD_IN_SECONDS

static double P_linear = 1.3 , D_linear = 0.0 , I_linear = 0.0   ; 				/* PID constants of linear x */
static double P_angular = -1.8 , D_angular = 0.01 , I_angular = -0.1   ;		/* PID constants of angular z */
double sum_linear;
double sum_angular;
ros::Publisher Throttle_rate_pub;
ros::Publisher Steering_rate_pub;

double errorAngularArray[1000];
int indexOf_errorAngularArray;
double errorLinearArray[1000];
int indexOf_errorLinearArray;
double lastLinearError;
double lastAngularError;
double LocVelLinearX;
double LocVelLinearY;
double LocVelAngularZ;
double WpdSpeedLinear;
double WpdSpeedAngular;

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
	ROS_INFO("LLC Init");
	sum_linear = 0;
	sum_angular = 0;
	LocVelLinearX = 0;
	LocVelLinearY = 0;
	LocVelAngularZ = 0;
	WpdSpeedLinear = 0;
	WpdSpeedAngular = 0;
	for(int i = 0 ; i < 1000 ; i++)
	  {
	    errorLinearArray[i] = 0;
	    errorAngularArray[i] = 0;
	  }
	Event e("EndOfInit");
	events.raiseEvent(e);
	return TaskResult::SUCCESS();
}



void dynamic_Reconfiguration_callback(llc::ControlParamsConfig &config, uint32_t level) {

	P_linear=config.linearVelocity_P;
	I_linear=config.linearVelocity_I;
	D_linear=config.linearVelocity_D;
	P_angular=config.angularVelocity_P;
	I_angular=config.angularVelocity_I;
	D_angular=config.angularVelocity_D;
}

double calcIntegral_linearError(double currError)
{
  sum_linear -= errorLinearArray[indexOf_errorLinearArray];
  sum_linear += currError;

  errorLinearArray[indexOf_errorLinearArray] = currError;

  if(++indexOf_errorLinearArray == LENGTH_OF_RECORD_IN_FRAMES)
          indexOf_errorLinearArray = 0;

  return sum_linear*DT;
}

double calcDiferencial_linearError(double currError)
{
  double diff = currError - lastLinearError;
  double result = diff / DT;
  lastLinearError = currError;
  return result;
}

double calcIntegral_angularError(double currError)
{
  sum_angular -= errorAngularArray[indexOf_errorAngularArray];
  sum_angular += currError;

  errorAngularArray[indexOf_errorAngularArray] = currError;
  indexOf_errorAngularArray++;

  if(indexOf_errorAngularArray == LENGTH_OF_RECORD_IN_FRAMES)
          indexOf_errorAngularArray = 0;

  return sum_angular*DT;
}

double calcDiferencial_angularError(double currError)
{
  double diff = currError - lastAngularError;
  double result = diff / DT;
  lastLinearError = currError;
  return result;
}

void cb_LocVelpcityUpdate(geometry_msgs::TwistStamped msg)
{
  LocVelLinearX = msg.twist.linear.x;
  LocVelLinearY = msg.twist.linear.y;

  LocVelAngularZ = msg.twist.angular.z;
}

void cb_WpdSpeedLinear(geometry_msgs::TwistStamped msg)
{
  WpdSpeedLinear = msg.twist.linear.x;
  WpdSpeedAngular = msg.twist.angular.z;
}

void pubThrottkeAndSteering()
{
    double locVelLin = sqrt((LocVelLinearX * LocVelLinearX) + (LocVelLinearY * LocVelLinearY));
    double linearError = WpdSpeedLinear - locVelLin;

    double linearEffortCMD = P_linear * linearError + I_linear* calcIntegral_linearError(linearError) + D_linear * calcDiferencial_linearError(linearError);

    std_msgs::Float64 msglinearEffortCMD;
    msglinearEffortCMD.data = linearEffortCMD;
    Throttle_rate_pub.publish(msglinearEffortCMD);


    double angularError = WpdSpeedAngular - LocVelAngularZ;
    double angularEffortCMD = P_angular * angularError + I_angular* calcIntegral_angularError(angularError) + D_angular * calcDiferencial_angularError(angularError) ;

    std_msgs::Float64 msgAngularEffortCMD;
    msgAngularEffortCMD.data = angularEffortCMD;
    Steering_rate_pub.publish(msgAngularEffortCMD);

}

TaskResult state_READY(string id, const CallContext& context, EventQueue& events){

	ros::NodeHandle n;

        Throttle_rate_pub = n.advertise<std_msgs::Float64>("/LLC/EFFORTS/Throttle", 100);
        Steering_rate_pub = n.advertise<std_msgs::Float64>("/LLC/EFFORTS/Steering", 100);

        ros::Subscriber locVel= n.subscribe("/LOC/Velocity" , 10, cb_LocVelpcityUpdate);
        ros::Subscriber wpdSpeed = n.subscribe("/WPD/Speed" , 10, cb_WpdSpeedLinear);

        ROS_INFO("LLC Ready");

        ros::Rate rate(100);
        while(ros::ok())
        {
          pubThrottkeAndSteering();
          rate.sleep();
          ros::spinOnce();
        }

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
