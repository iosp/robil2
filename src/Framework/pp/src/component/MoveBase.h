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
	void on_goal(const geometry_msgs::PoseStamped& robil_goal);
	void on_map(const config::PP::sub::Map& map);

	void calculate_goal();

protected:
	config::PP::sub::GlobalPath gotten_path;
	bool gp_defined;
	config::PP::sub::Location gotten_location;
	bool gl_defined;

protected:
	ros::Publisher goalPublisher;
	ros::Publisher mapPublisher;
};

#endif /* MOVEBASE_H_ */
