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

  ros::Subscriber * _sub_PosAttVel;
  ros::Subscriber * _sub_Teleoperation;
  ros::Subscriber * _sub_StatusData;
  ros::Subscriber * _sub_MissionStatus;
  ros::Subscriber * _sub_Map;
  ros::Subscriber * _sub_LocalPathPlan;
  ros::Subscriber * _sub_TrottleEffort;
  ros::Subscriber * _sub_StreeringEffort;


  ros::Publisher  * _pub_PositionUpdate;
  ros::Publisher  * _pub_MissionPlan;
  ros::Publisher  * _pub_Teleoperation;
  ros::Publisher  * _pub_IEDDetectionEvent;
  ros::Publisher  * _pub_IEDLocation;


public:
	RosComm(ComponentMain* comp,int argc,char** argv);
	virtual ~RosComm();
	std::string fetchParam(std::string compName,std::string neededTopic,std::string type);


	void PosAttVelCallback(const robil2_msgs::String::ConstPtr &msg);
	void TeleoperationCallback(const robil2_msgs::String::ConstPtr &msg);
	void StatusDataCallback(const robil2_msgs::String::ConstPtr &msg);
	void MissionStatusCallback(const robil2_msgs::String::ConstPtr &msg);
	void MapCallback(const robil2_msgs::String::ConstPtr &msg);
	void LocalPathPlanCallback(const robil2_msgs::String::ConstPtr &msg);
	void TrottleEffortCallback(const robil2_msgs::String::ConstPtr &msg);
	void IEDLocationCallback(const robil2_msgs::String::ConstPtr &msg);

	void publishPositionUpdate(robil2_msgs::String &msg);
	void publishMissionPlan(robil2_msgs::String &msg);
	void publishTeleoperation(robil2_msgs::String &msg);
	void publishIEDDetectionEvent(robil2_msgs::String &msg);
	void publishIEDLocation(robil2_msgs::String &msg);
};


#endif /* ROSCOMM_H_ */
