/*
 * Tester.h
 *
 *  Created on: Mar 6, 2014
 *      Author: dan
 */

#ifndef TESTER_H_
#define TESTER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <Geometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <map>
#include <set>
#include <vector>
#include <sstream>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

class Tester {
public:
	Tester();
	virtual ~Tester();

	void test1_init();
	void test1_step();


	nav_msgs::OccupancyGrid map;
	nav_msgs::Path path;
	geometry_msgs::Pose pose;
	nav_msgs::Path move;

	ros::NodeHandle node;
	ros::Publisher pub_path;
	ros::Publisher pub_map;
	ros::Publisher pub_location;
	ros::Publisher pub_command;
};

class Checker{
public:
	Checker();
	virtual ~Checker();

	void on_goal(const geometry_msgs::PoseStamped::ConstPtr& msg);

	ros::NodeHandle node;
	ros::Subscriber sub_on_goal;
};

#endif /* TESTER_H_ */
