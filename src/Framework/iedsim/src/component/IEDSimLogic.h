/*
 * IEDSimLogic.h
 *
 *  Created on: Apr 9, 2014
 *      Author: userws1
 */
#include <ros/ros.h>

#ifndef IEDSIMLOGIC_H_
#define IEDSIMLOGIC_H_

class IEDSimLogic {
	ros::NodeHandle inner_nh;
	ros::ServiceClient spawm_model;
	ros::ServiceClient move_model;
public:
	bool m_isSet;
	float m_x;
	float m_y;
	float m_z;
	float m_roll;
	float m_pitch;
	float m_yaw;


	void setAtLocation(float x,float y,float z);
	bool isPoseWithinRadius(float x,float y,float z);
	IEDSimLogic();
	virtual ~IEDSimLogic();
};

#endif /* IEDSIMLOGIC_H_ */
