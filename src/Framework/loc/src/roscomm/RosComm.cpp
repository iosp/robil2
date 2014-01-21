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
  ros::init(argc,argv,"LOC_node");
  _nh=new ros::NodeHandle;
  _comp=comp;

   _sub_PosAttVel=
       new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"LOC","PosAttVel","sub"), 1, &RosComm::PosAttVelCallback,this));
  _sub_PositionUpdate=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"LOC","PositionUpdate","sub"), 1, &RosComm::PositionUpdateCallback,this));

  _pub_PosAttVel=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"LOC","PosAttVel","pub"),1));

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



