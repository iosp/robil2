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

  ros::Subscriber * _sub_ExecuteWorkSequenceCommand;
  ros::Subscriber * _sub_PosAttVel;
  ros::Subscriber * _sub_WiresLengths;

  ros::Publisher  * _pub_BladePosition;
  ros::Publisher  * _pub_TrottleEffort;
  ros::Publisher  * _pub_SteeringEffort;
  ros::Publisher  * _pub_JointsEffort;


public:
	RosComm(ComponentMain* comp,int argc,char** argv);
	virtual ~RosComm();
	std::string fetchParam(std::string compName,std::string neededTopic,std::string type);


	void ExecuteWorkSequenceCommandCallback(const std_msgs::String::ConstPtr &msg);
        void PosAttVelCallback(const std_msgs::String::ConstPtr &msg);
        void WiresLengthsCallback(const std_msgs::String::ConstPtr &msg);

	void publishBladePosition(std_msgs::String &msg);
        void publishTrottleEffort(std_msgs::String &msg);
        void publishSteeringEffort(std_msgs::String &msg);
        void publishJointsEffort(std_msgs::String &msg);
};


#endif /* ROSCOMM_H_ */
