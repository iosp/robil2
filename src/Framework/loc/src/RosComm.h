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
  ros::Subscriber * _sub_PositionUpdate;

  ros::Publisher  * _pub_PosAttVel;

public:
	RosComm(ComponentMain* comp,int argc,char** argv);
	virtual ~RosComm();
	std::string fetchParam(std::string compName,std::string neededTopic,std::string type);


	void PosAttVelCallback(const robil2_msgs::String::ConstPtr &msg);
	void PositionUpdateCallback(const robil2_msgs::String::ConstPtr &msg);

	void publishPosAttVel(robil2_msgs::String &msg);
};


#endif /* ROSCOMM_H_ */
