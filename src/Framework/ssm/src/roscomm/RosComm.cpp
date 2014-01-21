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
  ros::init(argc,argv,"SSM_node");
  _nh=new ros::NodeHandle;
  _comp=comp;

  ros::Subscriber * _sub_MissionStatus=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"SSM","MissionStatus","sub"), 1, &RosComm::MissionStatusCallback,this));

  ros::Publisher  * _pub_StatusData=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"SSM","StatusData","pub"),1));

}

RosComm::~RosComm()
{
	// TODO Auto-generated destructor stub
}

void RosComm::MissionStatusCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleMissionStatus(*msg);
}

void RosComm::publishStatusData(std_msgs::String &msg)
{
  _pub_StatusData->publish(msg);
}

