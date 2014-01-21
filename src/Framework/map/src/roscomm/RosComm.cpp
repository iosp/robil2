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
  ros::init(argc,argv,"MAP_node");
  _nh=new ros::NodeHandle;
  _comp=comp;


  _sub_BladePosition=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"MAP","BladePosition","sub"),  1, &RosComm::BladePositionCallback,this));

  _sub_PosAttVel=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"MAP","PosAttVel","sub"), 1, &RosComm::PosAttVelCallback,this));

  _sub_Laser=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"MAP","Laser","sub"),   1, &RosComm::LaserCallback,this));

  _sub_Camera=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"MAP","Camera","sub"),  1, &RosComm::CameraCallback,this));

  _pub_Map=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"MAP","Map","pub"),   1));

  _pub_MiniMap=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"MAP","MiniMap","pub"),  1));



}

RosComm::~RosComm()
{
	// TODO Auto-generated destructor stub
}

void RosComm::BladePositionCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleBladePosition(*msg);
}

void RosComm::PosAttVelCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handlePosAttVel(*msg);
}
void RosComm::LaserCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleLaser(*msg);
}
void RosComm::CameraCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleCamera(*msg);
}


void RosComm::publishMap(std_msgs::String &msg)
{
 _pub_Map->publish(msg);
}

void RosComm::publishMiniMap(std_msgs::String &msg)
{
  _pub_MiniMap->publish(msg);
}
