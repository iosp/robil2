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
#include <ParameterTypes.h>
class ComponentMain;

class RosComm {
  ComponentMain   * _comp;

  ros::NodeHandle _nh;

  ros::Subscriber _sub_Map;
  ros::Subscriber _sub_MissionGlobalPath;
  ros::Subscriber _sub_IEDPosAtt;
  ros::Subscriber _sub_PosAttVel;
  ros::Subscriber _sub_RPPPath;

  ros::Publisher  _pub_LocalPathPlan;


public:
	RosComm(ComponentMain* comp,int argc,char** argv);
	virtual ~RosComm();

        void MapCallback(const config::PP::sub::Map::ConstPtr &msg);
        void MissionGlobalPathCallback(const config::PP::sub::MissionGlobalPath::ConstPtr &msg);
        void IEDPosAttCallback(const config::PP::sub::IEDPosAtt::ConstPtr &msg);
        void PosAttVelCallback(const config::PP::sub::PosAttVel::ConstPtr &msg);
        void RPPPathCallback(const config::PP::sub::RPPPath::ConstPtr &msg);

        void publishLocalPathPlan( config::PP::pub::LocalPathPlan &msg);
};


#endif /* ROSCOMM_H_ */
