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
  ros::init(argc,argv,"VO_node");
  _nh=new ros::NodeHandle;
  _comp=comp;


  _sub_Camera=
      new ros::Subscriber(_nh->subscribe(fetchParam("VO","Camera","sub"), 1, &RosComm::CameraCallback,this));

  _sub_INS=
      new ros::Subscriber(_nh->subscribe(fetchParam("VO","INS","sub"), 1, &RosComm::INSCallback,this));

  _sub_TF=
      new ros::Subscriber(_nh->subscribe(fetchParam("VO","TF","sub"), 1, &RosComm::TFCallback,this));

  _pub_PosAttVel=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam("VO","PosAttVel","pub"),1));

}

RosComm::~RosComm()
{
	// TODO Auto-generated destructor stub
}


void RosComm::CameraCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleCamera(*msg);
}

void RosComm::INSCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleINS(*msg);
}

void RosComm::TFCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleTF(*msg);
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
