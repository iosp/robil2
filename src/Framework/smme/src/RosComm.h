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

  ros::Subscriber * _sub_TrottleEffort;
  ros::Subscriber * _sub_SteeringEffort;
  ros::Subscriber * _sub_MissionPlan;
  ros::Subscriber * _sub_StatusData;

  ros::Publisher  * _pub_MissionStatus;
  ros::Publisher  * _pub_MissionGlobalPath;
  ros::Publisher  * _pub_IEDPosAtt;
  ros::Publisher  * _pub_ExecuteWorkSequenceCommand;

public:
	RosComm(ComponentMain* comp,int argc,char** argv);
	virtual ~RosComm();
	std::string fetchParam(std::string compName,std::string neededTopic,std::string type);


	void TrottleEffortCallback(const std_msgs::String::ConstPtr &msg);
	void IEDLocationCallback(const std_msgs::String::ConstPtr &msg);
        void MissionPlanCallback(const std_msgs::String::ConstPtr &msg);
        void StatusDataCallback(const std_msgs::String::ConstPtr &msg);

	void publishMissionStatus(std_msgs::String &msg);
        void publishMissionGlobalPath(std_msgs::String &msg);
        void publishIEDPosAtt(std_msgs::String &msg);
        void publishExecuteWorkSequenceCommand(std_msgs::String &msg);

};


#endif /* ROSCOMM_H_ */
