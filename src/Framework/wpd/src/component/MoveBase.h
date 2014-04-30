/*
 * MoveBase.h
 *
 *  Created on: Apr 30, 2014
 *      Author: dan
 */

#ifndef MOVEBASE_H_
#define MOVEBASE_H_

#include <iostream>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>

#include <ParameterHandler.h>
#include <ParameterTypes.h>

class ComponentMain;
class MoveBase {
public:
	MoveBase(ComponentMain* comp);
	virtual ~MoveBase();

	ComponentMain* comp;

	void on_position_update(const config::PP::sub::Location& location);
	ros::Subscriber sub_location;
	ros::Subscriber sub_location_cov;
	ros::Subscriber sub_speed;
	void on_sub_loc(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void on_sub_loc_cov(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void on_sub_speed(const geometry_msgs::Twist::ConstPtr& msg);
	void on_speed(const geometry_msgs::Twist& msg);
	config::PP::sub::Location last_location;
	void resend();
	boost::thread* resend_thread;
};

#endif /* MOVEBASE_H_ */
