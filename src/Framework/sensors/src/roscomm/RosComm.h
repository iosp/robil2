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

  ros::Publisher  * _pub_Sensor_SICK;
  ros::Publisher  * _pub_Sensor_IBEO;
  ros::Publisher  * _pub_Sensor_CAM_R;
  ros::Publisher  * _pub_Sensor_CAM_L;
  ros::Publisher  * _pub_Sensor_WIRE;
  ros::Publisher  * _pub_Sensor_INSGPS;

public:
	RosComm(ComponentMain* comp,int argc,char** argv);
	virtual ~RosComm();

	void publishSensor_SICK(std_msgs::String &msg);
	void publishSensor_IBEO(std_msgs::String &msg);
	void publishSensor_CAM_R(std_msgs::String &msg);
	void publishSensor_CAM_L(std_msgs::String &msg);
	void publishSensor_WIRE(std_msgs::String &msg);
	void publishSensor_INSGPS(std_msgs::String &msg);
};


#endif /* ROSCOMM_H_ */
