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
  ros::init(argc,argv,"VO_node");
  _nh=new ros::NodeHandle;
  _comp=comp;


  _sub_Camera=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"VO","Camera","sub"), 1, &RosComm::CameraCallback,this));

  _sub_INS=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"VO","INS","sub"), 1, &RosComm::INSCallback,this));

  _sub_TF=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"VO","TF","sub"), 1, &RosComm::TFCallback,this));

  _pub_PosAttVel=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"VO","PosAttVel","pub"),1));

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
