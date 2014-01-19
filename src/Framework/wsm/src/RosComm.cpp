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
  ros::init(argc,argv,"WSM_node");
  _nh=new ros::NodeHandle;
  _comp=comp;

  _sub_ExecuteWorkSequenceCommand=
      new ros::Subscriber(_nh->subscribe(fetchParam("WSM","ExecuteWorkSequenceCommand","sub"), 1, &RosComm::ExecuteWorkSequenceCommandCallback,this));

  _sub_PosAttVel=
      new ros::Subscriber(_nh->subscribe(fetchParam("WSM","PosAttVel","sub"), 1, &RosComm::PosAttVelCallback,this));

  _sub_WiresLengths=
      new ros::Subscriber(_nh->subscribe(fetchParam("WSM","WiresLengths","sub"), 1, &RosComm::WiresLengthsCallback,this));

  _pub_BladePosition=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("WSM","BladePosition","pub"),1));

  _pub_TrottleEffort=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("WSM","TrottleEffort","pub"),1));

  _pub_SteeringEffort=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("WSM","SteeringEffort","pub"),1));

  _pub_JointsEffort=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("WSM","JointsEffort","pub"),1));

}

RosComm::~RosComm()
{
  // TODO Auto-generated destructor stub
}

void RosComm::ExecuteWorkSequenceCommandCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handleExecuteWorkSequenceCommand(*msg);
}

void RosComm::PosAttVelCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handlePosAttVel(*msg);
}

void RosComm::WiresLengthsCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handleWiresLengths(*msg);
}

void RosComm::publishBladePosition(robil2_msgs::String &msg)
{
  _pub_BladePosition->publish(msg);
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
