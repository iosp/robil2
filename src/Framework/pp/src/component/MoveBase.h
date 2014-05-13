/*
 * MoveBase.h
 *
 *  Created on: Mar 5, 2014
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

using namespace std;

class MoveBase {
public:
	MoveBase();
	virtual ~MoveBase();

	void on_position_update(const config::PP::sub::Location& location);
	void on_path(const config::PP::sub::GlobalPath& goal_path);
	void on_path(const nav_msgs::Path& goal_path);
	void on_goal(const geometry_msgs::PoseStamped& robil_goal);
	void on_map(const config::PP::sub::Map& map);
	void on_map(const nav_msgs::OccupancyGrid& map);

	void calculate_goal();
	bool all_data_defined()const;

protected:
	config::PP::sub::GlobalPath gotten_path;
	bool gp_defined;
	nav_msgs::Path gotten_nav_path;
	bool gnp_defined;
	config::PP::sub::Location gotten_location;
	bool gl_defined;

protected:
	ros::Publisher goalPublisher;
	ros::Publisher mapPublisher;
	boost::recursive_mutex mtx;

//FOR TEST ONLY
public:
	ros::Subscriber sub_map;
	ros::Subscriber sub_path;
	ros::Subscriber sub_location;
	ros::Subscriber sub_location_cov;
	ros::Subscriber sub_commands;
	void on_sub_map(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void on_sub_path(const nav_msgs::Path::ConstPtr& msg);
	void on_sub_loc(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void on_sub_loc_cov(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void on_sub_commands(const std_msgs::String::ConstPtr& msg);

};

#endif /* MOVEBASE_H_ */
