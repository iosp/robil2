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

  ros::Subscriber *_sub_Sensor_SICK;
  ros::Subscriber *_sub_Sensor_IBEO;
  ros::Subscriber *_sub_Sensor_CAM_R;
  ros::Subscriber *_sub_Sensor_CAM_L;


public:
	RosComm(ComponentMain* comp,int argc,char** argv);
	virtual ~RosComm();
	std::string fetchParam(std::string compName,std::string neededTopic,std::string type);


	void Sensor_SICKCallback(const robil2_msgs::String::ConstPtr &msg);
	void Sensor_IBEOCallback(const robil2_msgs::String::ConstPtr &msg);
        void Sensor_CAM_RCallback(const robil2_msgs::String::ConstPtr &msg);
        void Sensor_CAM_LCallback(const robil2_msgs::String::ConstPtr &msg);
};


#endif /* ROSCOMM_H_ */
