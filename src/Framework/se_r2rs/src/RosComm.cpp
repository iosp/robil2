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
  ros::init(argc,argv,"SE_R2RS_node");
  _nh=new ros::NodeHandle;
  _comp=comp;

  _sub_Sensor_SICK=
      new ros::Subscriber(_nh->subscribe(fetchParam("SE_R2RS","Sensor_SICK","sub"), 1, &RosComm::Sensor_SICKCallback,this));

  _sub_Sensor_IBEO=
      new ros::Subscriber(_nh->subscribe(fetchParam("SE_R2RS","Sensor_IBEO","sub"), 1, &RosComm::Sensor_IBEOCallback,this));

  _sub_Sensor_CAM_R=
      new ros::Subscriber(_nh->subscribe(fetchParam("SE_R2RS","Sensor_CAM_R","sub"), 1, &RosComm::Sensor_CAM_RCallback,this));

  _sub_Sensor_CAM_L=
      new ros::Subscriber(_nh->subscribe(fetchParam("SE_R2RS","Sensor_CAM_L","sub"), 1, &RosComm::Sensor_CAM_LCallback,this));


}

RosComm::~RosComm()
{
	// TODO Auto-generated destructor stub
}

void RosComm::Sensor_SICKCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleSensor_SICK(*msg);
}

void RosComm::Sensor_IBEOCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleSensor_IBEO(*msg);
}

void RosComm::Sensor_CAM_RCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleSensor_CAM_R(*msg);
}

void RosComm::Sensor_CAM_LCallback(const std_msgs::String::ConstPtr &msg)
{
  _comp->handleSensor_CAM_L(*msg);
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
