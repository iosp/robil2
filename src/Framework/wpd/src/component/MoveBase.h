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


#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>


#ifndef HEARTBEAT_FREQUANCY
#define HEARTBEAT_FREQUANCY 2 //Hz
#endif

#ifndef HEARTBEAT_FREQUENCY
#define HEARTBEAT_FREQUENCY 2 //Hz
#endif


class ComponentMain;
class MoveBase {
private:
	geometry_msgs::Twist last_move_base_vel;
	geometry_msgs::Twist min_real_vel;
	geometry_msgs::Twist last_real_vel;
	double pub_frequancy; // Hz
	bool last_move_base_vel_ok;
	bool last_real_vel_ok;


public:
	MoveBase(ComponentMain* comp);
	virtual ~MoveBase();

	ComponentMain* comp;

	void on_position_update(const geometry_msgs::PoseWithCovarianceStamped& location);
	ros::Subscriber sub_location;
	ros::Subscriber sub_location_cov;
	ros::Subscriber sub_speed;
	void on_sub_loc(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void on_sub_loc_cov(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void on_sub_speed(const geometry_msgs::Twist::ConstPtr& msg);
	void on_speed(const geometry_msgs::Twist& msg);
	geometry_msgs::PoseWithCovarianceStamped last_location;
	void resend();
	boost::thread* resend_thread;


	void set_last_move_base_vel (const geometry_msgs::Twist & new_last_vel);
	void set_last_real_vel (const geometry_msgs::TwistStamped & new_real_vel);
	geometry_msgs::Twist get_last_move_base_vel ();
	geometry_msgs::Twist get_last_real_vel ();
	void set_min_real_vel (const geometry_msgs::Twist & new_min_real_vel);
	geometry_msgs::Twist get_min_real_vel ();
	void set_pub_frequancy (const double new_pub_frequancy);
	double get_pub_frequancy ();
	void running ();
	void load_param ();

};






#endif /* MOVEBASE_H_ */
