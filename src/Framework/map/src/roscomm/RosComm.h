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

  ros::Subscriber * _sub_BladePosition;
  ros::Subscriber * _sub_PosAttVel;
  ros::Subscriber * _sub_Laser;
  ros::Subscriber * _sub_Camera;

  ros::Publisher  * _pub_Map;
  ros::Publisher  * _pub_MiniMap;

public:
	RosComm(ComponentMain* comp,int argc,char** argv);
	virtual ~RosComm();


	void BladePositionCallback(const std_msgs::String::ConstPtr &msg);
	void PosAttVelCallback(const std_msgs::String::ConstPtr &msg);
	void LaserCallback(const std_msgs::String::ConstPtr &msg);
	void CameraCallback(const std_msgs::String::ConstPtr &msg);

	void publishMap(std_msgs::String &msg);
        void publishMiniMap(std_msgs::String &msg);
};


#endif /* ROSCOMM_H_ */
