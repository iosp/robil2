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

  ros::Subscriber *_sub_LocalPathPlan;
  ros::Subscriber *_sub_MiniMap;
  ros::Subscriber *_sub_PosAttVel;

  ros::Publisher  *_pub_RPPPath;

public:
	RosComm(ComponentMain* comp,int argc,char** argv);
	virtual ~RosComm();
	std::string fetchParam(std::string compName,std::string neededTopic,std::string type);


	void LocalPathPlanCallback(const std_msgs::String::ConstPtr &msg);
	void MiniMapCallback(const std_msgs::String::ConstPtr &msg);
        void PosAttVelCallback(const std_msgs::String::ConstPtr &msg);

	void publishRPPPath(std_msgs::String &msg);
};


#endif /* ROSCOMM_H_ */
