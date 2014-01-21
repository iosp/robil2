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

  ros::Subscriber *_sub_RPPPath;
  ros::Subscriber *_sub_PosAttVel;

  ros::Publisher  *_pub_TrottleEffort;
  ros::Publisher  *_pub_SteeringEffort;

public:
	RosComm(ComponentMain* comp,int argc,char** argv);
	virtual ~RosComm();

	void RPPPathCallback(const std_msgs::String::ConstPtr &msg);
	void PosAttVelCallback(const std_msgs::String::ConstPtr &msg);

	void publishTrottleEffort(std_msgs::String &msg);
        void publishSteeringEffort(std_msgs::String &msg);
};


#endif /* ROSCOMM_H_ */
