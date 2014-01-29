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
  _comp=comp;

  _sub_Map=
      ros::Subscriber(_nh.subscribe(fetchParam(&_nh,"PP","Map","sub"), 10, &RosComm::MapCallback,this));

  _sub_MissionGlobalPath=
      ros::Subscriber(_nh.subscribe(fetchParam(&_nh,"PP","MissionGlobalPath","sub"), 10, &RosComm::MissionGlobalPathCallback,this));

  _sub_IEDPosAtt=
      ros::Subscriber(_nh.subscribe(fetchParam(&_nh,"PP","IEDPosAtt","sub"), 10, &RosComm::IEDPosAttCallback,this));

  _sub_PosAttVel=
      ros::Subscriber(_nh.subscribe(fetchParam(&_nh,"PP","PosAttVel","sub"), 10, &RosComm::PosAttVelCallback,this));

  _sub_RPPPath=
      ros::Subscriber(_nh.subscribe(fetchParam(&_nh,"PP","RPPPath","sub"), 10, &RosComm::RPPPathCallback,this));

  _pub_LocalPathPlan=
      ros::Publisher(_nh.advertise<std_msgs::String>(fetchParam(&_nh,"PP","LocalPathPlan","pub"),10));

}

RosComm::~RosComm()
{

}


void RosComm::MapCallback(const config::PP::sub::Map::ConstPtr &msg)
{
  _comp->handleMap(*msg);
}
void RosComm::MissionGlobalPathCallback(const config::PP::sub::Map::ConstPtr &msg)
{
  _comp->handleMissionGlobalPath(*msg);
}
void RosComm::IEDPosAttCallback(const config::PP::sub::IEDPosAtt::ConstPtr &msg)
{
  _comp->handleIEDPosAtt(*msg);
}
void RosComm::PosAttVelCallback(const config::PP::sub::PosAttVel::ConstPtr &msg)
{
  _comp->handlePosAttVel(*msg);
}
void RosComm::RPPPathCallback(const config::PP::sub::RPPPath::ConstPtr &msg)
{
  _comp->handleRPPPath(*msg);
}

void RosComm::publishLocalPathPlan( config::PP::pub::LocalPathPlan &msg)
{
  _pub_LocalPathPlan.publish(msg);
}


