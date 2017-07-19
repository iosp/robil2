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
#include <actionlib_msgs/GoalStatusArray.h>

#include <rosgraph_msgs/Log.h>
typedef rosgraph_msgs::Log LogMessage;

#define DIAGNOSTIC_TOPIC_NAME "/diagnostics"
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include "GC.h"

#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <robil_msgs/Path.h>
#include <robil_msgs/Map.h>

#ifndef HEARTBEAT_FREQUANCY
#define HEARTBEAT_FREQUANCY 2 //Hz
#endif

#ifndef HEARTBEAT_FREQUENCY
#define HEARTBEAT_FREQUENCY 2 //Hz
#endif


using namespace std;

class ComponentMain;

class MoveBase {
public:
	MoveBase(ComponentMain* comp);
	virtual ~MoveBase();

	void on_position_update(const geometry_msgs::PoseWithCovarianceStamped& location);
	void on_path(const robil_msgs::Path& goal_path);
	void on_path(const nav_msgs::Path& goal_path);
	void on_goal(const geometry_msgs::PoseStamped& robil_goal);
	void on_map(const robil_msgs::Map& map);
	void on_map(const nav_msgs::OccupancyGrid& map);

	void on_nav_path(const nav_msgs::Path& goal_pat);

	void calculate_goal();
	bool all_data_defined()const;
	void notify_path_is_finished(bool success)const;

	void cancel(bool clear_last_goals = false);
	void activate();
	void deactivate(bool clear_last_goals = false);

protected:
	bool is_active;
	robil_msgs::Path gotten_path;
	bool gp_defined;
	nav_msgs::Path gotten_nav_path;
	bool gnp_defined;
	geometry_msgs::PoseWithCovarianceStamped gotten_location;
	bool gl_defined;
	bool is_canceled;
	bool is_path_calculated;

protected:
	ros::Publisher goalPublisher;
	ros::Publisher originalGoalPublisher;
	ros::Publisher goalCancelPublisher;
	ros::Publisher mapPublisher;
	ros::Publisher fakeLaserPublisher;
	ros::Publisher globalPathPublisher;
	ros::Publisher selectedPathPublisher;
	ros::Publisher pathVisualizationPublisher;
	ros::Subscriber pathSubscriber;
	ros::Subscriber sub_log;
	ros::Subscriber moveBaseStatusSubscriber;
	ros::Subscriber globalCostmapSubscriber;
	ros::Subscriber speedSubscriber;
	ros::Publisher diagnosticPublisher;

	boost::recursive_mutex mtx;
	ComponentMain* comp;

	RobilGC::GoalCalculator goal_calculator;


	void on_log_message(const LogMessage::ConstPtr& msg);
	void on_log_message(int type, string message);
	void on_error_from_move_base();
	void stop_navigation(bool success);
	void on_move_base_status(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);


	std::map<string, size_t> unvisited_index;
	size_t get_unvisited_index(string path_id);
	void remove_memory_about_path(string path_id);
	void update_unvisited_index(string path_id, size_t new_index);

	string last_diagnostic_message_id;
	void diagnostic_publish_new_goal(const string& path_id, const geometry_msgs::PoseStamped& goal, size_t goal_index, const geometry_msgs::PoseWithCovarianceStamped& gotten_location);

	void publish_global_gotten_path_visualization(nav_msgs::Path global_gotten_path);
	std::string init_path(std::vector<goal_calculator::Point_2d> & path);

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
