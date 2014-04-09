/*
 * IEDSimLogic.cpp
 *
 *  Created on: Apr 9, 2014
 *      Author: userws1
 */

#include "IEDSimLogic.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <sstream>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SetModelState.h>


IEDSimLogic::IEDSimLogic() :
	m_isSet(false),
	m_x(0),
	m_y(0),
	m_z(0),
	m_roll(0),
	m_pitch(0),
	m_yaw(0)
{
	spawm_model=inner_nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
	move_model=inner_nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
}

IEDSimLogic::~IEDSimLogic()
{
	// TODO Auto-generated destructor stub
}


void IEDSimLogic::setAtLocation(float x,float y,float z)
{
	m_x=x;
	m_y=y;
	m_z=z;

	if(m_isSet)
	{
		gazebo_msgs::SetModelState srv;
		srv.request.model_state.model_name="IEDSIM_IED";
		srv.request.model_state.pose.position.x=x;
		srv.request.model_state.pose.position.y=y;
		srv.request.model_state.pose.position.z=z;
		move_model.call(srv);
	}
	else
	{
		std::stringstream ss;
		ss << "<sdf version=\"1.4\">";
		ss << "<world name=\"default\">";
		ss << "<include>";
		ss << "<uri>model\:\/\/ied</uri>";
		ss << "</include>";
		ss << "</world>";
		ss << "</sdf>";
		gazebo_msgs::SpawnModel srv;
		srv.request.initial_pose.position.x=x;
		srv.request.initial_pose.position.y=y;
		srv.request.initial_pose.position.z=z;
		srv.request.model_name="IEDSIM_IED";
		srv.request.model_xml=ss.str();
		m_isSet=true;
		spawm_model.call(srv);
	}
}
bool IEDSimLogic::isPoseWithinRadius(float x,float y,float z)
{
	if( std::sqrt(std::pow(m_x-x,2)+std::pow(m_y-y,2)+std::pow(m_z-z,2))<=10)
	{
		return true;
	}
	return false;
}

