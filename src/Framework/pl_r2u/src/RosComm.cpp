/*
 * RosComm.cpp
 *
 *  Created on: Jan 16, 2014
 *      Author: yuval
 */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "RosComm.h"
#include "ComponentMain.h"
#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>

RosComm::RosComm(ComponentMain* comp,int argc,char** argv)
{
  ros::init(argc,argv,"PL_R2U_node");
  _nh=new ros::NodeHandle;
  _comp=comp;

  _sub_TrottleEffort=
      new ros::Subscriber(_nh->subscribe(fetchParam("PL_R2U","TrottleEffort","sub"), 1, &RosComm::TrottleEffortCallback,this));

  _sub_SteeringEffort=
      new ros::Subscriber(_nh->subscribe(fetchParam("PL_R2U","SteeringEffort","sub"), 1, &RosComm::StreeringEffortCallback,this));

  _sub_JointsEffort=
      new ros::Subscriber(_nh->subscribe(fetchParam("PL_R2U","JointsEffort","sub"), 1, &RosComm::JointsEffortCallback,this));


}

RosComm::~RosComm()
{
	// TODO Auto-generated destructor stub
}

void RosComm::TrottleEffortCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleTrottleEffort(*msg);
}

void RosComm::StreeringEffortCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleStreeringEffort(*msg);
}

void RosComm::JointsEffortCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleJointsEffort(*msg);
}

std::string RosComm::fetchParam(std::string compName,std::string neededTopic,std::string type)
{
  std::stringstream ss;
  ss<<compName<<"_"<<type<<"_"<<neededTopic;
  std::string answer;
  if(_nh->hasParam(ss.str()))
  {
    _nh->getParam(ss.str(),answer);
  }
  else
  {
    ROS_ERROR("%s not found on prarmeter server",ss.str().c_str());
    return "";
  }
  if(type.compare("sub")==0)
  {
    std::stringstream ss2;
    ss2<<answer<<"_"<<"pub"<<"_"<<neededTopic;
    if(_nh->hasParam(ss2.str()))
    {
      _nh->getParam(ss2.str(),answer);
    }
    else
    {
      ROS_ERROR("%s not found on prarmeter server",ss2.str().c_str());
      return "";
    }
  }
  return answer;
}
