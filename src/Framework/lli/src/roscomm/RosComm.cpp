/*
 * RosComm.cpp
 *
 *  Created on: Jan 16, 2014
 *      Author: yuval
 */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "RosComm.h"
#include "../component/ComponentMain.h"
#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>
#include "ParameterHandler.h"

RosComm::RosComm(ComponentMain* comp,int argc,char** argv)
{
  ros::init(argc,argv,"LLI_node");
  _nh=new ros::NodeHandle;
  _comp=comp;


  _sub_TrottleEffort=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"LLI","TrottleEffort","sub"),  1, &RosComm::TrottleEffortCallback,this));

  _sub_SteeringEffort=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"LLI","SteeringEffort","sub"), 1, &RosComm::SteeringEffortCallback,this));

  _sub_JointsEffort=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"LLI","JointsEffort","sub"),   1, &RosComm::JointsEffortCallback,this));

  _sub_Teleoperation=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"LLI","Teleoperation","sub"),  1, &RosComm::TeleoperationCallback,this));


  _pub_TrottleEffort=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"LLI","TrottleEffort","pub"),  1));

  _pub_SteeringEffort=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"LLI","SteeringEffort","pub"), 1));

  _pub_JointsEffort=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"LLI","JointsEffort","pub"),   1));

  _pub_Teleoperation=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"LLI","Teleoperation","pub"),  1));



}

RosComm::~RosComm()
{
	// TODO Auto-generated destructor stub
}

void RosComm::TrottleEffortCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleTrottleEffort(*msg);
}

void RosComm::SteeringEffortCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleSteeringEffort(*msg);
}
void RosComm::JointsEffortCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleJointsEffort(*msg);
}
void RosComm::TeleoperationCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleTeleoperation(*msg);
}

void RosComm::publishTrottleEffort(std_msgs::String &msg)
{
  _pub_TrottleEffort->publish(msg);
}

void RosComm::publishSteeringEffort(std_msgs::String &msg)
{
 _pub_SteeringEffort->publish(msg);
}

void RosComm::publishJointsEffort(std_msgs::String &msg)
{
 _pub_JointsEffort->publish(msg);
}

void RosComm::publishTeleoperation(std_msgs::String &msg)
{
  _pub_Teleoperation->publish(msg);
}

