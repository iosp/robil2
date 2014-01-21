/*
 * RosComm.cpp
 *
 *  Created on: Jan 16, 2014
 *      Author: yuval
 */
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "RosComm.h"
#include "../component/ComponentMain.h"
#include <stdio.h>
#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>
#include "ParameterHandler.h"

RosComm::RosComm(ComponentMain* comp,int argc,char** argv)
{
  ros::init(argc,argv,"IEDSIM_node");
  _nh=new ros::NodeHandle;
  _comp=comp;
  _sub_IEDDetectionEvent=new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"IEDSIM","IEDDetectionEvent","sub"), 1, &RosComm::IEDDetectionEventCallback,this));
  _sub_IEDLocation=new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"IEDSIM","IEDLocation","sub"), 1, &RosComm::IEDLocationCallback,this));

  _pub_IEDDetectionEvent=new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"IEDSIM","IEDDetectionEvent","pub"),1));
  _pub_IEDLocation=new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"IEDSIM","IEDLocation","pub"),1));
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



