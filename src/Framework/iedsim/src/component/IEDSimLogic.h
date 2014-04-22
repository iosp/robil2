/*
 * IEDSimLogic.h
 *
 *  Created on: Apr 9, 2014
 *      Author: userws1
 */
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#ifndef IEDSIMLOGIC_H_
#define IEDSIMLOGIC_H_

class IEDSimLogic {
	ros::NodeHandle inner_nh;
	ros::ServiceClient spawm_model;
	ros::ServiceClient move_model;
	ros::Subscriber check_model;
public:
	bool m_isSet;
	float m_x;
	float m_y;
	float m_z;
	float m_roll;
	float m_pitch;
	float m_yaw;
	int msg_counter;

	void updateLocationFromSim(const gazebo_msgs::ModelStates::ConstPtr &msg);
	void setAtLocation(float x,float y,float z);
	bool isPoseWithinRadius(float x,float y,float z);
	IEDSimLogic();
	virtual ~IEDSimLogic();
};

#endif /* IEDSIMLOGIC_H_ */
