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
  ros::init(argc,argv,"SENSORS_node");
  _nh=new ros::NodeHandle;
  _comp=comp;

  _pub_Sensor_SICK=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"SENSORS","Sensor_SICK","pub"),1));

  _pub_Sensor_IBEO=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"SENSORS","Sensor_IBEO","pub"),1));

  _pub_Sensor_CAM_R=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"SENSORS","Sensor_CAM_R","pub"),1));

  _pub_Sensor_CAM_L=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"SENSORS","Sensor_CAM_L","pub"),1));

  _pub_Sensor_WIRE=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"SENSORS","Sensor_WIRE","pub"),1));

  _pub_Sensor_INSGPS=
      new ros::Publisher(_nh->advertise<std_msgs::String>(fetchParam(_nh,"SENSORS","Sensor_INSGPS","pub"),1));

}

RosComm::~RosComm()
{
	// TODO Auto-generated destructor stub
}


void RosComm::publishSensor_SICK(std_msgs::String &msg)
{
  _pub_Sensor_SICK->publish(msg);
}
void RosComm::publishSensor_IBEO(std_msgs::String &msg)
{
  _pub_Sensor_IBEO->publish(msg);
}
void RosComm::publishSensor_CAM_R(std_msgs::String &msg)
{
  _pub_Sensor_CAM_R->publish(msg);
}
void RosComm::publishSensor_CAM_L(std_msgs::String &msg)
{
  _pub_Sensor_CAM_L->publish(msg);
}
void RosComm::publishSensor_WIRE(std_msgs::String &msg)
{
  _pub_Sensor_WIRE->publish(msg);
}
void RosComm::publishSensor_INSGPS(std_msgs::String &msg)
{
  _pub_Sensor_INSGPS->publish(msg);
}

