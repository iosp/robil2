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
  ros::init(argc,argv,"WPD_node");
  _nh=new ros::NodeHandle;
  _comp=comp;

  _sub_RPPPath=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"WPD","RPPPath","sub"), 1, &RosComm::RPPPathCallback,this));
  _sub_PosAttVel=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"WPD","PosAttVel","sub"), 1, &RosComm::PosAttVelCallback,this));

  _pub_TrottleEffort=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"WPD","TrottleEffort","pub"),1));
  _pub_SteeringEffort=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"WPD","SteeringEffort","pub"),1));
}

RosComm::~RosComm()
{
	// TODO Auto-generated destructor stub
}

void RosComm::RPPPathCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleRPPPath(*msg);
}
void RosComm::PosAttVelCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handlePosAttVel(*msg);
}

void RosComm::publishTrottleEffort(std_msgs::String &msg)
{
  _pub_TrottleEffort->publish(msg);
}
void RosComm::publishSteeringEffort(std_msgs::String &msg)
{
  _pub_SteeringEffort->publish(msg);
}


