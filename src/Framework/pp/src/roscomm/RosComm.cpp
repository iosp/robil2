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
  ros::init(argc,argv,"PP_node");
  _nh=new ros::NodeHandle;
  _comp=comp;


  _sub_Map=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"PP","Map","sub"), 1, &RosComm::MapCallback,this));

  _sub_MissionGlobalPath=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"PP","MissionGlobalPath","sub"), 1, &RosComm::MissionGlobalPathCallback,this));

  _sub_IEDPosAtt=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"PP","IEDPosAtt","sub"), 1, &RosComm::IEDPosAttCallback,this));

  _sub_PosAttVel=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"PP","PosAttVel","sub"), 1, &RosComm::PosAttVelCallback,this));

  _sub_RPPPath=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"PP","RPPPath","sub"), 1, &RosComm::RPPPathCallback,this));

  _pub_LocalPathPlan=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"PP","LocalPathPlan","pub"),1));

}

RosComm::~RosComm()
{
	// TODO Auto-generated destructor stub
}


void RosComm::MapCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleMap(*msg);
}
void RosComm::MissionGlobalPathCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleMissionGlobalPath(*msg);
}
void RosComm::IEDPosAttCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleIEDPosAtt(*msg);
}
void RosComm::PosAttVelCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handlePosAttVel(*msg);
}
void RosComm::RPPPathCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleRPPPath(*msg);
}

void RosComm::publishLocalPathPlan(std_msgs::String &msg)
{
  _pub_LocalPathPlan->publish(msg);
}


