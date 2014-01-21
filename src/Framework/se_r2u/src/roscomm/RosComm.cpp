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
  ros::init(argc,argv,"SE_R2U_node");
  _nh=new ros::NodeHandle;
  _comp=comp;

   _sub_Sensor_WIRE=
       new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"SE_R2U","Sensor_WIRE","sub"), 1, &RosComm::Sensor_WIRECallback,this));
  _sub_Sensor_INSGPS=
      new ros::Subscriber(_nh->subscribe(fetchParam(_nh,"SE_R2U","Sensor_INSGPS","sub"), 1, &RosComm::Sensor_INSGPSCallback,this));

}

RosComm::~RosComm()
{
	// TODO Auto-generated destructor stub
}

void RosComm::Sensor_WIRECallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleSensor_WIRE(*msg);
}

void RosComm::Sensor_INSGPSCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleSensor_INSGPS(*msg);
}
