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
  ros::init(argc,argv,"RPP_node");
  _nh=new ros::NodeHandle;
  _comp=comp;

  _sub_LocalPathPlan=
      new ros::Subscriber(_nh->subscribe(fetchParam("RPP","LocalPathPlan","sub"), 1, &RosComm::LocalPathPlanCallback,this));

  _sub_MiniMap=
      new ros::Subscriber(_nh->subscribe(fetchParam("RPP","MiniMap","sub"), 1, &RosComm::MiniMapCallback,this));

  _sub_PosAttVel=
      new ros::Subscriber(_nh->subscribe(fetchParam("RPP","PosAttVel","sub"), 1, &RosComm::PosAttVelCallback,this));


  _pub_RPPPath=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("RPP","RPPPath","pub"),1));

}

RosComm::~RosComm()
{
	// TODO Auto-generated destructor stub
}


void RosComm::LocalPathPlanCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handleLocalPathPlan(*msg);
}
void RosComm::MiniMapCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handleMiniMap(*msg);
}
void RosComm::PosAttVelCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handlePosAttVel(*msg);
}

void RosComm::publishRPPPath(robil2_msgs::String &msg)
{
  _pub_RPPPath->publish(msg);
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
