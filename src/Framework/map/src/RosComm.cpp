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
  ros::init(argc,argv,"MAP_node");
  _nh=new ros::NodeHandle;
  _comp=comp;


  _sub_BladePosition=
      new ros::Subscriber(_nh->subscribe(fetchParam("MAP","BladePosition","sub"),  1, &RosComm::BladePositionCallback,this));

  _sub_PosAttVel=
      new ros::Subscriber(_nh->subscribe(fetchParam("MAP","PosAttVel","sub"), 1, &RosComm::PosAttVelCallback,this));

  _sub_Laser=
      new ros::Subscriber(_nh->subscribe(fetchParam("MAP","Laser","sub"),   1, &RosComm::LaserCallback,this));

  _sub_Camera=
      new ros::Subscriber(_nh->subscribe(fetchParam("MAP","Camera","sub"),  1, &RosComm::CameraCallback,this));

  _pub_Map=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("MAP","Laser","pub"),   1));

  _pub_MiniMap=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("MAP","Camera","pub"),  1));



}

RosComm::~RosComm()
{
	// TODO Auto-generated destructor stub
}

void RosComm::BladePositionCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handleBladePosition(*msg);
}

void RosComm::PosAttVelCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handlePosAttVel(*msg);
}
void RosComm::LaserCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handleLaser(*msg);
}
void RosComm::CameraCallback(const robil2_msgs::String::ConstPtr &msg)
{
  _comp->handleCamera(*msg);
}


void RosComm::publishMap(robil2_msgs::String &msg)
{
 _pub_Map->publish(msg);
}

void RosComm::publishMiniMap(robil2_msgs::String &msg)
{
  _pub_MiniMap->publish(msg);
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
