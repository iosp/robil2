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
  ros::init(argc,argv,"SMME_node");
  _nh=new ros::NodeHandle;
  _comp=comp;



  _sub_TrottleEffort=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"SMME","IEDDetectionEvent","sub"), 1, &RosComm::TrottleEffortCallback,this));

  _sub_SteeringEffort=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"SMME","IEDLocation","sub"), 1, &RosComm::IEDLocationCallback,this));

  _sub_MissionPlan=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"SMME","MissionPlan","sub"), 1, &RosComm::MissionPlanCallback,this));

  _sub_StatusData=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"SMME","StatusData","sub"), 1, &RosComm::StatusDataCallback,this));


  _pub_MissionStatus=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"SMME","MissionStatus","pub"),1));
  _pub_MissionGlobalPath=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"SMME","MissionGlobalPath","pub"),1));

  _pub_IEDPosAtt=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"SMME","IEDPosAtt","pub"),1));

  _pub_ExecuteWorkSequenceCommand=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"SMME","ExecuteWorkSequenceCommand","pub"),1));


}

RosComm::~RosComm()
{
	// TODO Auto-generated destructor stub
}

void RosComm::TrottleEffortCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleIEDDetectionEvent(*msg);
}
void RosComm::IEDLocationCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleIEDLocation(*msg);
}
void RosComm::MissionPlanCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleMissionPlan(*msg);
}
void RosComm::StatusDataCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleStatusData(*msg);
}

void RosComm::publishMissionStatus(std_msgs::String &msg)
{
  _pub_MissionStatus->publish(msg);
}
void RosComm::publishMissionGlobalPath(std_msgs::String &msg)
{
  _pub_MissionGlobalPath->publish(msg);
}
void RosComm::publishIEDPosAtt(std_msgs::String &msg)
{
  _pub_IEDPosAtt->publish(msg);
}
void RosComm::publishExecuteWorkSequenceCommand(std_msgs::String &msg)
{
  _pub_ExecuteWorkSequenceCommand->publish(msg);
}

