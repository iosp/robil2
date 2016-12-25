/*
 * ParameterHandler.cpp
 *
 *  Created on: Jan 21, 2014
 *      Author: userws1
 */

#include "../include/ParameterHandler.h"
#include "ros/ros.h"

std::string fetchParam(ros::NodeHandle *_nh,std::string compName,std::string neededTopic,std::string type)
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
    ROS_ERROR("compName: %s, %s not found on prarmeter server",compName.c_str(),ss.str().c_str());
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
      ROS_ERROR("compName: %s, %s not found on prarmeter server",compName.c_str(),ss2.str().c_str());
      return "";
    }
  }
  return answer;
}
