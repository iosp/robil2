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


  ros::Subscriber * _sub_Sensor_SICK;
  ros::Subscriber * _sub_Sensor_IBEO;
  ros::Subscriber * _sub_Sensor_CAM_R;
  ros::Subscriber * _sub_Sensor_CAM_L;
  ros::Subscriber * _sub_Sensor_WIRE;
  ros::Subscriber * _sub_Sensor_INSGPS;

  ros::Publisher  * _pub_WiresLengths;
  ros::Publisher  * _pub_Camera;
  ros::Publisher  * _pub_Laser;
  ros::Publisher  * _pub_INS;
  ros::Publisher  * _pub_GPS;
  ros::Publisher  * _pub_TF;

public:
	RosComm(ComponentMain* comp,int argc,char** argv);
	virtual ~RosComm();
	std::string fetchParam(std::string compName,std::string neededTopic,std::string type);


        void Sensor_SICKCallback(const std_msgs::String::ConstPtr &msg);
        void Sensor_IBEOCallback(const std_msgs::String::ConstPtr &msg);
        void Sensor_CAM_RCallback(const std_msgs::String::ConstPtr &msg);
        void Sensor_CAM_LCallback(const std_msgs::String::ConstPtr &msg);
        void Sensor_WIRECallback(const std_msgs::String::ConstPtr &msg);
        void Sensor_INSGPSCallback(const std_msgs::String::ConstPtr &msg);

        void publishWiresLengths(std_msgs::String &msg);
        void publishCamera(std_msgs::String &msg);
        void publishLaser(std_msgs::String &msg);
        void publishINS(std_msgs::String &msg);
        void publishGPS(std_msgs::String &msg);
        void publishTF(std_msgs::String &msg);

};


#endif /* ROSCOMM_H_ */
