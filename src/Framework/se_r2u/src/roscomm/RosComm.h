/*
 * RosComm.h
 *
 *  Created on: Jan 16, 2014
 *      Author: yuval
 */

#ifndef ROSCOMM_H_
#define ROSCOMM_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>
class ComponentMain;

class RosComm {
  ComponentMain   * _comp;

  ros::NodeHandle * _nh;

  ros::Subscriber * _sub_Sensor_WIRE;
  ros::Subscriber * _sub_Sensor_INSGPS;


public:
	RosComm(ComponentMain* comp,int argc,char** argv);
	virtual ~RosComm();


	void Sensor_WIRECallback(const std_msgs::String::ConstPtr &msg);
	void Sensor_INSGPSCallback(const std_msgs::String::ConstPtr &msg);
};


#endif /* ROSCOMM_H_ */
