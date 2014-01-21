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

  ros::Subscriber * _sub_PosAttVel;
  ros::Subscriber * _sub_Teleoperation;
  ros::Subscriber * _sub_StatusData;
  ros::Subscriber * _sub_MissionStatus;
  ros::Subscriber * _sub_Map;
  ros::Subscriber * _sub_LocalPathPlan;
  ros::Subscriber * _sub_TrottleEffort;
  ros::Subscriber * _sub_SteeringEffort;


  ros::Publisher  * _pub_PositionUpdate;
  ros::Publisher  * _pub_MissionPlan;
  ros::Publisher  * _pub_Teleoperation;
  ros::Publisher  * _pub_IEDDetectionEvent;
  ros::Publisher  * _pub_IEDLocation;


public:
	RosComm(ComponentMain* comp,int argc,char** argv);
	virtual ~RosComm();


	void PosAttVelCallback(const std_msgs::String::ConstPtr &msg);
	void TeleoperationCallback(const std_msgs::String::ConstPtr &msg);
	void StatusDataCallback(const std_msgs::String::ConstPtr &msg);
	void MissionStatusCallback(const std_msgs::String::ConstPtr &msg);
	void MapCallback(const std_msgs::String::ConstPtr &msg);
	void LocalPathPlanCallback(const std_msgs::String::ConstPtr &msg);
	void TrottleEffortCallback(const std_msgs::String::ConstPtr &msg);
	void IEDLocationCallback(const std_msgs::String::ConstPtr &msg);

	void publishPositionUpdate(std_msgs::String &msg);
	void publishMissionPlan(std_msgs::String &msg);
	void publishTeleoperation(std_msgs::String &msg);
	void publishIEDDetectionEvent(std_msgs::String &msg);
	void publishIEDLocation(std_msgs::String &msg);
};


#endif /* ROSCOMM_H_ */
