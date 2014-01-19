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
  ros::init(argc,argv,"OCU_node");
  _nh=new ros::NodeHandle;
  _comp=comp;


  _sub_PosAttVel=
      new ros::Subscriber(_nh->subscribe(fetchParam("OCU","PosAttVel","sub"), 1, &RosComm::PosAttVelCallback,this));
  _sub_Teleoperation=
      new ros::Subscriber(_nh->subscribe(fetchParam("OCU","Teleoperation","sub"), 1, &RosComm::TeleoperationCallback,this));
  _sub_StatusData=
      new ros::Subscriber(_nh->subscribe(fetchParam("OCU","StatusData","sub"), 1, &RosComm::StatusDataCallback,this));
  _sub_MissionStatus=
      new ros::Subscriber(_nh->subscribe(fetchParam("OCU","MissionStatus","sub"), 1, &RosComm::MissionStatusCallback,this));

  _sub_Map=
      new ros::Subscriber(_nh->subscribe(fetchParam("OCU","Map","sub"), 1, &RosComm::MapCallback,this));

  _sub_LocalPathPlan=
      new ros::Subscriber(_nh->subscribe(fetchParam("OCU","LocalPathPlan","sub"), 1, &RosComm::LocalPathPlanCallback,this));

  _sub_IEDDetectionEvent=
      new ros::Subscriber(_nh->subscribe(fetchParam("OCU","IEDDetectionEvent","sub"), 1, &RosComm::IEDDetectionEventCallback,this));

  _sub_IEDLocation=
      new ros::Subscriber(_nh->subscribe(fetchParam("OCU","IEDLocation","sub"), 1, &RosComm::IEDLocationCallback,this));


  _pub_PositionUpdate=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("OCU","PositionUpdate","pub"),1));

  _pub_MissionPlan=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("OCU","MissionPlan","pub"),1));
  _pub_Teleoperation=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("OCU","Teleoperation","pub"),1));
  _pub_IEDDetectionEvent=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("OCU","IEDDetectionEvent","pub"),1));
  _pub_IEDLocation=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("OCU","IEDLocation","pub"),1));


}

RosComm::~RosComm()
{
	// TODO Auto-generated destructor stub
}



void RosComm::PosAttVelCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handlePosAttVel(*msg);
}

void RosComm::TeleoperationCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handleTeleoperation(*msg);
}
void RosComm::StatusDataCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handleStatusData(*msg);
}
void RosComm::MissionStatusCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handleMissionStatus(*msg);
}
void RosComm::MapCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handleMap(*msg);
}
void RosComm::LocalPathPlanCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handleLocalPathPlan(*msg);
}
void RosComm::IEDDetectionEventCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handleIEDDetectionEvent(*msg);
}
void RosComm::IEDLocationCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handleIEDLocation(*msg);
}

void RosComm::publishPositionUpdate(robil2_msgs::String &msg)
{
  _pub_PositionUpdate->publish(msg);
}
void RosComm::publishMissionPlan(robil2_msgs::String &msg)
{
  _pub_MissionPlan->publish(msg);
}
void RosComm::publishTeleoperation(robil2_msgs::String &msg)
{
  _pub_Teleoperation->publish(msg);
}
void RosComm::publishIEDDetectionEvent(robil2_msgs::String &msg)
{
  _pub_IEDDetectionEvent->publish(msg);
}
void RosComm::publishIEDLocation(robil2_msgs::String &msg)
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
