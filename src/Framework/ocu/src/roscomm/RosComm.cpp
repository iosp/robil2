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
  ros::init(argc,argv,"OCU_node");
  _nh=new ros::NodeHandle;
  _comp=comp;


  _sub_PosAttVel=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"OCU","PosAttVel","sub"), 1, &RosComm::PosAttVelCallback,this));
  _sub_Teleoperation=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"OCU","Teleoperation","sub"), 1, &RosComm::TeleoperationCallback,this));
  _sub_StatusData=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"OCU","StatusData","sub"), 1, &RosComm::StatusDataCallback,this));
  _sub_MissionStatus=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"OCU","MissionStatus","sub"), 1, &RosComm::MissionStatusCallback,this));

  _sub_Map=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"OCU","Map","sub"), 1, &RosComm::MapCallback,this));

  _sub_LocalPathPlan=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"OCU","LocalPathPlan","sub"), 1, &RosComm::LocalPathPlanCallback,this));

  _sub_TrottleEffort=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"OCU","IEDDetectionEvent","sub"), 1, &RosComm::TrottleEffortCallback,this));

  _sub_SteeringEffort=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"OCU","IEDLocation","sub"), 1, &RosComm::IEDLocationCallback,this));


  _pub_PositionUpdate=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"OCU","PositionUpdate","pub"),1));

  _pub_MissionPlan=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"OCU","MissionPlan","pub"),1));
  _pub_Teleoperation=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"OCU","Teleoperation","pub"),1));
  _pub_IEDDetectionEvent=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"OCU","IEDDetectionEvent","pub"),1));
  _pub_IEDLocation=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"OCU","IEDLocation","pub"),1));


}

RosComm::~RosComm()
{
	// TODO Auto-generated destructor stub
}



void RosComm::PosAttVelCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handlePosAttVel(*msg);
}

void RosComm::TeleoperationCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleTeleoperation(*msg);
}
void RosComm::StatusDataCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleStatusData(*msg);
}
void RosComm::MissionStatusCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleMissionStatus(*msg);
}
void RosComm::MapCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleMap(*msg);
}
void RosComm::LocalPathPlanCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleLocalPathPlan(*msg);
}
void RosComm::TrottleEffortCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleIEDDetectionEvent(*msg);
}
void RosComm::IEDLocationCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleIEDLocation(*msg);
}

void RosComm::publishPositionUpdate(std_msgs::String &msg)
{
  _pub_PositionUpdate->publish(msg);
}
void RosComm::publishMissionPlan(std_msgs::String &msg)
{
  _pub_MissionPlan->publish(msg);
}
void RosComm::publishTeleoperation(std_msgs::String &msg)
{
  _pub_Teleoperation->publish(msg);
}
void RosComm::publishIEDDetectionEvent(std_msgs::String &msg)
{
  _pub_IEDDetectionEvent->publish(msg);
}
void RosComm::publishIEDLocation(std_msgs::String &msg)
{
  _pub_IEDLocation->publish(msg);
}



