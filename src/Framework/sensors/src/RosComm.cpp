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
  ros::init(argc,argv,"SENSORS_node");
  _nh=new ros::NodeHandle;
  _comp=comp;

  _pub_Sensor_SICK=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("SENSORS","Sensor_SICK","pub"),1));

  _pub_Sensor_IBEO=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("SENSORS","Sensor_IBEO","pub"),1));

  _pub_Sensor_CAM_R=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("SENSORS","Sensor_CAM_R","pub"),1));

  _pub_Sensor_CAM_L=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("SENSORS","Sensor_CAM_L","pub"),1));

  _pub_Sensor_WIRE=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("SENSORS","Sensor_WIRE","pub"),1));

  _pub_Sensor_INSGPS=
      new ros::Publisher(_nh->advertise<robil2_msgs::String>(fetchParam("SENSORS","Sensor_INSGPS","pub"),1));

}

RosComm::~RosComm()
{
	// TODO Auto-generated destructor stub
}


void RosComm::publishSensor_SICK(robil2_msgs::String &msg)
{
  _pub_Sensor_SICK->publish(msg);
}
void RosComm::publishSensor_IBEO(robil2_msgs::String &msg)
{
  _pub_Sensor_IBEO->publish(msg);
}
void RosComm::publishSensor_CAM_R(robil2_msgs::String &msg)
{
  _pub_Sensor_CAM_R->publish(msg);
}
void RosComm::publishSensor_CAM_L(robil2_msgs::String &msg)
{
  _pub_Sensor_CAM_L->publish(msg);
}
void RosComm::publishSensor_WIRE(robil2_msgs::String &msg)
{
  _pub_Sensor_WIRE->publish(msg);
}
void RosComm::publishSensor_INSGPS(robil2_msgs::String &msg)
{
  _pub_Sensor_INSGPS->publish(msg);
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
