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
  ros::init(argc,argv,"SMME_node");
  _nh=new ros::NodeHandle;
  _comp=comp;



  _sub_TrottleEffort=
      new ros::Subscriber(_nh->subscribe(fetchParam("SMME","IEDDetectionEvent","sub"), 1, &RosComm::TrottleEffortCallback,this));

  _sub_StreeringEffort=
      new ros::Subscriber(_nh->subscribe(fetchParam("SMME","IEDLocation","sub"), 1, &RosComm::IEDLocationCallback,this));

  _sub_MissionPlan=
      new ros::Subscriber(_nh->subscribe(fetchParam("SMME","MissionPlan","sub"), 1, &RosComm::MissionPlanCallback,this));

  _sub_StatusData=
      new ros::Subscriber(_nh->subscribe(fetchParam("SMME","StatusData","sub"), 1, &RosComm::StatusDataCallback,this));


  _pub_MissionStatus=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("SMME","MissionStatus","pub"),1));
  _pub_MissionGlobalPath=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("SMME","MissionGlobalPath","pub"),1));

  _pub_IEDPosAtt=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("SMME","IEDPosAtt","pub"),1));

  _pub_ExecuteWorkSequenceCommand=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("SMME","ExecuteWorkSequenceCommand","pub"),1));


}

RosComm::~RosComm()
{
	// TODO Auto-generated destructor stub
}

void RosComm::TrottleEffortCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handleIEDDetectionEvent(*msg);
}
void RosComm::IEDLocationCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handleIEDLocation(*msg);
}
void RosComm::MissionPlanCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handleMissionPlan(*msg);
}
void RosComm::StatusDataCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handleStatusData(*msg);
}

void RosComm::publishMissionStatus(robil2_msgs::String &msg)
{
  _pub_MissionStatus->publish(msg);
}
void RosComm::publishMissionGlobalPath(robil2_msgs::String &msg)
{
  _pub_MissionGlobalPath->publish(msg);
}
void RosComm::publishIEDPosAtt(robil2_msgs::String &msg)
{
  _pub_IEDPosAtt->publish(msg);
}
void RosComm::publishExecuteWorkSequenceCommand(robil2_msgs::String &msg)
{
  _pub_ExecuteWorkSequenceCommand->publish(msg);
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
