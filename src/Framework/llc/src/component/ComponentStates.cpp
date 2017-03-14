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

#define DT 0.1
#define LENGTH_OF_RECORD_IN_SECONDS 3
#define LENGTH_OF_RECORD_IN_FRAMES (1 / DT) * LENGTH_OF_RECORD_IN_SECONDS
#define SIZE_OF_WPD_INTEGRAL 3
static double P_linear, D_linear, I_linear;    /* PID constants of linear x */
static double P_angular, D_angular, I_angular; /* PID constants of angular z */
static double linearnormalizedValuer;
double sum_linear;
double sum_angular;

double errorAngularArray[1000];
int indexOf_errorAngularArray;
double errorLinearArray[1000];
int indexOf_errorLinearArray;
double wpdCmdLinearArray[SIZE_OF_WPD_INTEGRAL];
int indexOf_wpdCmdLinearArray;
double wpdCmdAngularArray[SIZE_OF_WPD_INTEGRAL];
int indexOf_wpdCmdAngularArray;
double lastLinearError;
double lastAngularError;
double LocVelLinearX;
double LocVelLinearY;
double LocVelAngularZ;
double sumOfWpdSpeedLinear;
double sumOfWpdSpeedAngular;
double WpdSpeedLinearLimit=2;
double WpdSpeedAngularLimit=1.6;
double WpdSpeedLinear;
double WpdSpeedAngular;
double currentYaw;
double currentVelocity;
double wpdSpeedTimeInMilli;
double linearFactor;
double angularFactor;

class Params : public CallContextParameters
{
public:
  ComponentMain *comp;
  Params(ComponentMain *comp) : comp(comp) {}
  std::string str() const { return ""; }
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
      FSM_TRANSITIONS
      {
	FSM_ON_EVENT("/llc/Standby", FSM_NEXT(STANDBY));
      }
    }
    FSM_STATE(STANDBY)
    {
      FSM_CALL_TASK(STANDBY)
      FSM_TRANSITIONS
      {
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

TaskResult state_OFF(string id, const CallContext &context, EventQueue &events)
{
  ROS_INFO("LLC OFF");
  return TaskResult::SUCCESS();
}

TaskResult state_INIT(string id, const CallContext &context, EventQueue &events)
{
  ROS_INFO("LLC Init");
  sum_linear = 0;
  sum_angular = 0;
  LocVelLinearX = 0;
  LocVelLinearY = 0;
  LocVelAngularZ = 0;
  WpdSpeedLinear = 0;
  WpdSpeedAngular = 0;
  indexOf_errorAngularArray = 0;
  indexOf_errorLinearArray = 0;
  indexOf_wpdCmdAngularArray = 0;
  indexOf_wpdCmdLinearArray = 0;
  sumOfWpdSpeedLinear = 0;
  sumOfWpdSpeedAngular = 0;
  currentYaw = 0;
  currentVelocity = 0;
  linearnormalizedValuer = 1;
  wpdSpeedTimeInMilli = 0;

  for (int i = 0; i < 1000; i++)
  {
    errorLinearArray[i] = 0;
    errorAngularArray[i] = 0;
  }
  for (int i = 0; i < SIZE_OF_WPD_INTEGRAL; i++)
  {
    wpdCmdLinearArray[i] = 0;
    wpdCmdAngularArray[i] = 0;
  }
  Event e("EndOfInit");
  events.raiseEvent(e);
  return TaskResult::SUCCESS();
}

void dynamic_Reconfiguration_callback(llc::ControlParamsConfig &config, uint32_t level)
{

  P_linear = config.linearVelocity_P;
  I_linear = config.linearVelocity_I;
  D_linear = config.linearVelocity_D;
  P_angular = config.angularVelocity_P;
  I_angular = config.angularVelocity_I;
  D_angular = config.angularVelocity_D;

  linearFactor = config.linearFactor;
  angularFactor = config.angularFactor;
}
double normalizedValue(double ValueToNormalize,double lim) //A normalizing function that clips the value to -1,1 range
{
  double value=0;
  if (ValueToNormalize > lim)
    value = lim;
  else if (ValueToNormalize < -lim)
    value = -lim;
  else
    value = ValueToNormalize;
	return value;
}
void cb_currentYaw(geometry_msgs::PoseWithCovarianceStamped msg)
{
  double roll, pitch, yaw = 0;
  tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  tf::Matrix3x3 rot_mat(q);
  rot_mat.getEulerYPR(yaw, pitch, roll);

  currentYaw = yaw;
}
double EMAa_prior=0;
double EMAl_prior=0;
double calcIntegral_linearError(double currError)
{
  sum_linear += currError*DT;
  sum_linear=normalizedValue(sum_linear,100);
  return sum_linear; //normalizing to prevent illogical integral accumulation that would be meaningless in the given output range(-1,1)
}

double calcDiferencial_linearError(double currError)
{
  double EMAl=EMAl_prior+0.3*(currError-EMAl_prior);
  double diff = EMAl - EMAl_prior;
  double result = diff / DT;
  EMAl_prior=EMAl;
  lastLinearError = currError;
  return result;
}

double calcIntegral_angularError(double currError)
{
  sum_angular += currError/DT;
  sum_angular=normalizedValue(sum_angular,1000);
  return sum_angular; //normalizing to prevent illogical integral accumulation
}

double calcDiferencial_angularError(double currError)
{
  double EMAa=EMAa_prior+0.3*(currError-EMAa_prior);
  double diff = EMAa - EMAa_prior;
  double result = diff / DT;
  lastAngularError = currError;
  EMAa_prior=EMAa;
  return result;
}

void cb_LocVelocityUpdate(geometry_msgs::TwistStamped msg)
{
  LocVelLinearX = msg.twist.linear.x;
  LocVelLinearY = msg.twist.linear.y;

  LocVelAngularZ = msg.twist.angular.z;

  double V_normal = sqrt((LocVelLinearX * LocVelLinearX) + (LocVelLinearY * LocVelLinearY));
  if (V_normal > 0.01)
  {
    double x_normal = LocVelLinearX / V_normal;
    double y_noraml = LocVelLinearY / V_normal;
    double theta = atan2(y_noraml, x_normal);
    if (abs(theta - currentYaw) - 3.0 / 8.0 * M_PI < 0.01 or abs(theta - currentYaw)>5)
    {
      currentVelocity = V_normal;
    }
    else if (abs(theta - currentYaw) - 5.0 / 8.0 * M_PI < 0.01)
    {
      currentVelocity = 0;
    }
    else
      currentVelocity = -V_normal;
  }
}

void cb_WpdSpeed(geometry_msgs::TwistStamped msg)
{
  wpdSpeedTimeInMilli = ros::Time::now().toSec() * 1000; // toSec() return seconds.milliSecconds

  sumOfWpdSpeedLinear -= wpdCmdLinearArray[indexOf_wpdCmdLinearArray];
  sumOfWpdSpeedLinear += normalizedValue(msg.twist.linear.x,WpdSpeedLinearLimit);
  sumOfWpdSpeedAngular -= wpdCmdAngularArray[indexOf_wpdCmdAngularArray];
  sumOfWpdSpeedAngular += normalizedValue(msg.twist.angular.z,WpdSpeedAngularLimit);

  wpdCmdLinearArray[indexOf_wpdCmdLinearArray] = normalizedValue(msg.twist.linear.x,WpdSpeedLinearLimit);
  wpdCmdAngularArray[indexOf_wpdCmdAngularArray] = normalizedValue(msg.twist.angular.z,WpdSpeedAngularLimit);
  if (++indexOf_wpdCmdLinearArray >= SIZE_OF_WPD_INTEGRAL)
    indexOf_wpdCmdLinearArray = 0;
  if (++indexOf_wpdCmdAngularArray >= SIZE_OF_WPD_INTEGRAL)
    indexOf_wpdCmdAngularArray = 0;

  WpdSpeedLinear = sumOfWpdSpeedLinear / SIZE_OF_WPD_INTEGRAL;
  WpdSpeedAngular = sumOfWpdSpeedAngular / SIZE_OF_WPD_INTEGRAL;
}

void getThrottleAndSteering(double &throttle, double &angular)
{
  double RosTimeNowInMilli = ros::Time::now().toSec() * 1000; // toSec() return seconds.milliSecconds

  //if there is no WPD command as long as  500ms, give the zero command
  if (wpdSpeedTimeInMilli + 500 - RosTimeNowInMilli <= 0)
  {
    WpdSpeedLinear = 0;
    WpdSpeedAngular = 0;
  }
  // printf( "lin = [%3.2f] ang = [%3.2f]\n",currentVelocity,LocVelAngularZ);

  //Calculatin linear baseline  using the Y=a*x^b power function calculated from trials.
  double baseLinCommand = 0.7*pow(fabs(WpdSpeedLinear),0.491)*WpdSpeedLinear/fabs(WpdSpeedLinear);
  //PID
  double linearError = (linearFactor * WpdSpeedLinear) - currentVelocity;
  double linearEffortCMD =baseLinCommand + P_linear * linearError + I_linear * calcIntegral_linearError(linearError) + D_linear * calcDiferencial_linearError(linearError);
  //clamping output
  if(WpdSpeedLinear>=0&&linearEffortCMD<baseLinCommand*0.7)linearEffortCMD=baseLinCommand*0.8;
  else if(WpdSpeedLinear>=0&&linearEffortCMD>baseLinCommand*1.5)linearEffortCMD=baseLinCommand*1.5;
  else if(WpdSpeedLinear<0&&linearEffortCMD>baseLinCommand*0.7)linearEffortCMD=baseLinCommand*0.8;
  else if(WpdSpeedLinear<0&&linearEffortCMD<baseLinCommand*1.5)linearEffortCMD=baseLinCommand*1.5;
  if(WpdSpeedLinear==0)linearEffortCMD=0;
  throttle = normalizedValue(linearEffortCMD, 1); //values larger than 1 are meaningless to the platform.

  //Calculatin angular baseline given 2 working states, static and moving.
  double baseAngCommand;
  double absAng =fabs(WpdSpeedAngular); //fabs(​WpdSpeedAngular);
  if(WpdSpeedLinear<0.15)
    baseAngCommand =0.904*pow(absAng,0.21)*WpdSpeedAngular/absAng;
  else
    baseAngCommand =0.884*pow(absAng,0.532)*WpdSpeedAngular/absAng;
  //PID
  double angularError = (angularFactor * WpdSpeedAngular) - LocVelAngularZ;
  double angularEffortCMD =baseAngCommand+ P_angular * angularError + I_angular * calcIntegral_angularError(angularError) + D_angular * calcDiferencial_angularError(angularError);
  //clamping output
  if(WpdSpeedAngular>=0&&angularEffortCMD<baseAngCommand*0.8)angularEffortCMD=baseAngCommand*0.9;
  else if(WpdSpeedAngular<0&&angularEffortCMD>baseAngCommand*0.8)angularEffortCMD=baseAngCommand*0.9;
  else if(WpdSpeedAngular<0&&angularEffortCMD<baseAngCommand*1.4)angularEffortCMD=baseAngCommand*1.4;
  else if(WpdSpeedAngular>0&&angularEffortCMD>baseAngCommand*1.4)angularEffortCMD=baseAngCommand*1.4;
  else if(WpdSpeedAngular==0)angularEffortCMD=0;
  angular = normalizedValue(angularEffortCMD, 1); //values larger than 1 are meaningless to the platform.
  
}
TaskResult state_READY(string id, const CallContext &context, EventQueue &events)
{

  ros::NodeHandle n;

  ros::Subscriber locVel = n.subscribe("/LOC/Velocity", 10, cb_LocVelocityUpdate);
  ros::Subscriber locPose = n.subscribe("/LOC/Pose", 10, cb_currentYaw);
  ros::Subscriber wpdSpeed = n.subscribe("/WPD/Speed", 10, cb_WpdSpeed);

  ROS_INFO("LLC Ready");

  ros::Rate rate(100);
  while (ros::ok())
  {
    double throttle, angular;
    getThrottleAndSteering(throttle, angular);

    std_msgs::Float64 msglinearEffortCMD;
    msglinearEffortCMD.data = throttle;
    context.parameters<Params>().comp->publishEffortsTh(msglinearEffortCMD);

    std_msgs::Float64 msgAngularEffortCMD;
    msgAngularEffortCMD.data = angular;
    context.parameters<Params>().comp->publishEffortsSt(msgAngularEffortCMD);

    rate.sleep();
    ros::spinOnce();
  }

  return TaskResult::SUCCESS();
}
TaskResult state_STANDBY(string id, const CallContext &context, EventQueue &events)
{

  ROS_INFO("LLC Standby");

  return TaskResult::SUCCESS();
}

void runComponent(int argc, char **argv, ComponentMain &component)
{

  ros::init(argc, argv, "llc");
  ros_decision_making_init(argc, argv);
  RosEventQueue events;
  CallContext context;
  context.createParameters(new Params(&component));
  LocalTasks::registration("OFF", state_OFF);
  LocalTasks::registration("INIT", state_INIT);
  LocalTasks::registration("READY", state_READY);
  LocalTasks::registration("STANDBY", state_STANDBY);

  dynamic_reconfigure::Server<llc::ControlParamsConfig> server;
  dynamic_reconfigure::Server<llc::ControlParamsConfig>::CallbackType f;

  f = boost::bind(&dynamic_Reconfiguration_callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Starting llc...");
  Fsmllc(&context, &events);
}
