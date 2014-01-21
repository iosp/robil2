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
  ros::init(argc,argv,"RPP_node");
  _nh=new ros::NodeHandle;
  _comp=comp;

  _sub_LocalPathPlan=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"RPP","LocalPathPlan","sub"), 1, &RosComm::LocalPathPlanCallback,this));

  _sub_MiniMap=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"RPP","MiniMap","sub"), 1, &RosComm::MiniMapCallback,this));

  _sub_PosAttVel=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"RPP","PosAttVel","sub"), 1, &RosComm::PosAttVelCallback,this));


  _pub_RPPPath=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"RPP","RPPPath","pub"),1));

}

RosComm::~RosComm()
{
	// TODO Auto-generated destructor stub
}


void RosComm::LocalPathPlanCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleLocalPathPlan(*msg);
}
void RosComm::MiniMapCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleMiniMap(*msg);
}
void RosComm::PosAttVelCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handlePosAttVel(*msg);
}

void RosComm::publishRPPPath(std_msgs::String &msg)
{
  _pub_RPPPath->publish(msg);
}

