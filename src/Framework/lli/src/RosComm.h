/*
 * RosComm.h
 *
 *  Created on: Jan 16, 2014
 *      Author: yuval
 */

#ifndef ROSCOMM_H_
#define ROSCOMM_H_

#include <ros/ros.h>
#include <robil2_msgs/String.h>
#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>
class ComponentMain;

class RosComm {
  ComponentMain   * _comp;

  ros::NodeHandle * _nh;

  ros::Subscriber * _sub_TrottleEffort;
  ros::Subscriber * _sub_SteeringEffort;
  ros::Subscriber * _sub_JointsEffort;
  ros::Subscriber * _sub_Teleoperation;

  ros::Publisher  * _pub_TrottleEffort;
  ros::Publisher  * _pub_SteeringEffort;
  ros::Publisher  * _pub_JointsEffort;
  ros::Publisher  * _pub_Teleoperation;

public:
	RosComm(ComponentMain* comp,int argc,char** argv);
	virtual ~RosComm();
	std::string fetchParam(std::string compName,std::string neededTopic,std::string type);


	void TrottleEffortCallback(const robil2_msgs::String::ConstPtr &msg);
	void SteeringEffortCallback(const robil2_msgs::String::ConstPtr &msg);
	void JointsEffortCallback(const robil2_msgs::String::ConstPtr &msg);
	void TeleoperationCallback(const robil2_msgs::String::ConstPtr &msg);

	void publishTrottleEffort(robil2_msgs::String &msg);
        void publishSteeringEffort(robil2_msgs::String &msg);
        void publishJointsEffort(robil2_msgs::String &msg);
        void publishTeleoperation(robil2_msgs::String &msg);
};


#endif /* ROSCOMM_H_ */
