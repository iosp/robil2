/*
 * RosComm.cpp
 *
 *  Created on: Jan 16, 2014
 *      Author: yuval
 */
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "RosComm.h"
#include "ComponentMain.h"
#include <stdio.h>
#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>

RosComm::RosComm(ComponentMain* comp,int argc,char** argv)
{
  ros::init(argc,argv,"IEDSIM_node");
  _nh=new ros::NodeHandle;
  _comp=comp;
  _sub_IEDDetectionEvent=new ros::Subscriber(_nh->subscribe(fetchParam("IEDSIM","IEDDetectionEvent","sub"), 1, &RosComm::IEDDetectionEventCallback,this));
  _sub_IEDLocation=new ros::Subscriber(_nh->subscribe(fetchParam("IEDSIM","IEDLocation","sub"), 1, &RosComm::IEDLocationCallback,this));

  _pub_IEDDetectionEvent=new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam("IEDSIM","IEDDetectionEvent","pub"),1));
  _pub_IEDLocation=new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam("IEDSIM","IEDLocation","pub"),1));
}

RosComm::~RosComm()
{
	// TODO Auto-generated destructor stub
}

void RosComm::IEDDetectionEventCallback(const std_msgs::String::ConstPtr &msg)
{

  _comp->handleIEDDetectionEvent(*msg);
}

void RosComm::IEDLocationCallback(const std_msgs::String::ConstPtr &msg)
{

  _comp->handleIEDLocation(*msg);
}

void RosComm::publishIEDDetectionEvent(std_msgs::String &msg)
{
  _pub_IEDDetectionEvent->publish(msg);
}

void RosComm::publishIEDLocation(std_msgs::String &msg)
{
  _pub_IEDLocation->publish(msg);
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
