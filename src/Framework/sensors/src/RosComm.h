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

  ros::Publisher  * _pub_Sensor_SICK;
  ros::Publisher  * _pub_Sensor_IBEO;
  ros::Publisher  * _pub_Sensor_CAM_R;
  ros::Publisher  * _pub_Sensor_CAM_L;
  ros::Publisher  * _pub_Sensor_WIRE;
  ros::Publisher  * _pub_Sensor_INSGPS;

public:
	RosComm(ComponentMain* comp,int argc,char** argv);
	virtual ~RosComm();
	std::string fetchParam(std::string compName,std::string neededTopic,std::string type);


	void publishSensor_SICK(robil2_msgs::String &msg);
	void publishSensor_IBEO(robil2_msgs::String &msg);
	void publishSensor_CAM_R(robil2_msgs::String &msg);
	void publishSensor_CAM_L(robil2_msgs::String &msg);
	void publishSensor_WIRE(robil2_msgs::String &msg);
	void publishSensor_INSGPS(robil2_msgs::String &msg);
};


#endif /* ROSCOMM_H_ */
