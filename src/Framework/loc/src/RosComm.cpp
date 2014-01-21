/*
 * RosComm.cpp
 *
 *  Created on: Jan 16, 2014
 *      Author: yuval
 */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "RosComm.h"
#include "ComponentMain.h"
#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>

RosComm::RosComm(ComponentMain* comp,int argc,char** argv)
{
  ros::init(argc,argv,"LOC_node");
  _nh=new ros::NodeHandle;
  _comp=comp;

   _sub_PosAttVel=
       new ros::Subscriber(_nh->subscribe(fetchParam("LOC","PosAttVel","sub"), 1, &RosComm::PosAttVelCallback,this));
  _sub_PositionUpdate=
      new ros::Subscriber(_nh->subscribe(fetchParam("LOC","PositionUpdate","sub"), 1, &RosComm::PositionUpdateCallback,this));

  _pub_PosAttVel=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam("LOC","PosAttVel","pub"),1));

}

RosComm::~RosComm()
{
	// TODO Auto-generated destructor stub
}

void RosComm::PosAttVelCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handlePosAttVel(*msg);
}

void RosComm::PositionUpdateCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handlePositionUpdate(*msg);
}

void RosComm::publishPosAttVel(std_msgs::String &msg)
{
  _pub_PosAttVel->publish(msg);
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
