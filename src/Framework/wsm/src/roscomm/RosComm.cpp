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
  ros::init(argc,argv,"WSM_node");
  _nh=new ros::NodeHandle;
  _comp=comp;

  _sub_ExecuteWorkSequenceCommand=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"WSM","ExecuteWorkSequenceCommand","sub"), 1, &RosComm::ExecuteWorkSequenceCommandCallback,this));

  _sub_PosAttVel=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"WSM","PosAttVel","sub"), 1, &RosComm::PosAttVelCallback,this));

  _sub_WiresLengths=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"WSM","WiresLengths","sub"), 1, &RosComm::WiresLengthsCallback,this));

  _pub_BladePosition=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"WSM","BladePosition","pub"),1));

  _pub_TrottleEffort=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"WSM","TrottleEffort","pub"),1));

  _pub_SteeringEffort=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"WSM","SteeringEffort","pub"),1));

  _pub_JointsEffort=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"WSM","JointsEffort","pub"),1));

}

RosComm::~RosComm()
{
  // TODO Auto-generated destructor stub
}

void RosComm::ExecuteWorkSequenceCommandCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleExecuteWorkSequenceCommand(*msg);
}

void RosComm::PosAttVelCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handlePosAttVel(*msg);
}

void RosComm::WiresLengthsCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleWiresLengths(*msg);
}

void RosComm::publishBladePosition(std_msgs::String &msg)
{
  _pub_BladePosition->publish(msg);
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

