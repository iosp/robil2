/*
 * RosComm.cpp
 *
 *  Created on: Jan 16, 2014
 *      Author: yuval
 */
#include <ros/ros.h>
#include <robil2_msgs/String.h>
#include "RosComm.h"
#include "ComponentMain.h"
#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>

RosComm::RosComm(ComponentMain* comp,int argc,char** argv)
{
  ros::init(argc,argv,"LLI_node");
  _nh=new ros::NodeHandle;
  _comp=comp;


  _sub_TrottleEffort=
      new ros::Subscriber(_nh->subscribe(fetchParam("LLI","TrottleEffort","sub"),  1, &RosComm::TrottleEffortCallback,this));

  _sub_SteeringEffort=
      new ros::Subscriber(_nh->subscribe(fetchParam("LLI","SteeringEffort","sub"), 1, &RosComm::SteeringEffortCallback,this));

  _sub_JointsEffort=
      new ros::Subscriber(_nh->subscribe(fetchParam("LLI","JointsEffort","sub"),   1, &RosComm::JointsEffortCallback,this));

  _sub_Teleoperation=
      new ros::Subscriber(_nh->subscribe(fetchParam("LLI","Teleoperation","sub"),  1, &RosComm::TeleoperationCallback,this));


  _pub_TrottleEffort=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("LLI","TrottleEffort","pub"),  1));

  _pub_SteeringEffort=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("LLI","SteeringEffort","pub"), 1));

  _pub_JointsEffort=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("LLI","JointsEffort","pub"),   1));

  _pub_Teleoperation=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("LLI","Teleoperation","pub"),  1));



}

RosComm::~RosComm()
{
	// TODO Auto-generated destructor stub
}

void RosComm::TrottleEffortCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handleTrottleEffort(*msg);
}

void RosComm::SteeringEffortCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handleSteeringEffort(*msg);
}
void RosComm::JointsEffortCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handleJointsEffort(*msg);
}
void RosComm::TeleoperationCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handleTeleoperation(*msg);
}

void RosComm::publishTrottleEffort(robil2_msgs::String &msg)
{
  _pub_TrottleEffort->publish(msg);
}

void RosComm::publishSteeringEffort(robil2_msgs::String &msg)
{
 _pub_SteeringEffort->publish(msg);
}

void RosComm::publishJointsEffort(robil2_msgs::String &msg)
{
 _pub_JointsEffort->publish(msg);
}

void RosComm::publishTeleoperation(robil2_msgs::String &msg)
{
  _pub_Teleoperation->publish(msg);
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
