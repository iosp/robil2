/*
 * GoalCalculator.h
 *
 *  Created on: May 7, 2017
 *      Author: assaf
 */

#ifndef NAVEX_NAVIGATION_INCLUDE_NAVEX_NAVIGATION_GOALCALCULATOR_H_
#define NAVEX_NAVIGATION_INCLUDE_NAVEX_NAVIGATION_GOALCALCULATOR_H_

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <vector>
#include <string>
#include <fstream>
#include <boost/shared_ptr.hpp>

#include "GoalCalculatorMath.h"

using namespace std;

namespace RobilGC
{
	typedef vector<Point_2d> Path;

	struct Config
	{
		double same_point_eps;
		double robot_is_near_distance;

		Config():same_point_eps(1E-5), robot_is_near_distance(1.5){}
	};

	struct Logger
	{
		#define GC_LOG_DIR string("/tmp/gc_logs")
		#define GC_LOG_PREFIX string("gc")

		int			_num_of_logs;
		int			_current_log_file;
		std::string	_log;

		void init(int log_files);
		void advance();
		void log(const Map & map, const Point_2d & robot, Path & path);
	};

	#define INPUT
	#define OUTPUT

	bool is_last_point(const Path &path, const Point_2d &p);
	bool is_last_point(const Path &path, const Index &p);

	bool is_passed(
			INPUT
			const Path& path,
			const Point_2d& robot,
			const Point_2d* prev,
			const Point_2d& point,
			const Point_2d* post
		);
	bool is_passed(
			INPUT
			const Path& path,
			const Point_2d& robot,
			const Index prev,
			const Point_2d& point,
			const Index post
		);
	bool is_passed(
			INPUT
			const Path& path,
			const Point_2d& robot,
			const Index prev,
			const Index point,
			const Index post
		);

	Index search_nearest(const Path &path, const Point_2d &robot);

	bool search_accessible_point(INPUT const Map &map, const Point_2d &b, const Point_2d &e,
									 OUTPUT    Point_2d &free_point);
	bool search_accessible_point(INPUT const Map &map, const Path &path, const Index b, const Index e,
									 OUTPUT Point_2d &free_point, Index &waypoint);
	bool search_accessible_point(INPUT const Map &map, const Point_2d &point,
									 OUTPUT    Point_2d &free_point);

	bool get_goal(INPUT    const Map &map, const Path &path, const Point_2d &robot,
					  OUTPUT Point_2d &goal, Index &waypoint);

	class GoalCalculator
	{
		struct WorldContext
		{
			nav_msgs::OccupancyGrid	_map;
			Point_2d				_robot;
			nav_msgs::Path			_path;
		};

		WorldContext			_world;
		boost::shared_ptr<Map>	_cell_map;
		Path					_path;
		Logger					_logger;
		Index					_wpi;

		void updateCellMap();

	public:
		GoalCalculator(int log_files):_wpi(0){_logger.init(log_files);}

		bool updateMap(nav_msgs::OccupancyGrid & new_map, geometry_msgs::PoseWithCovarianceStamped & new_pose);
		void updatePath(nav_msgs::Path & gotten_path);

		bool get_goal(geometry_msgs::PoseStamped & goal, Index & wpi);
	};
}

#endif /* NAVEX_NAVIGATION_INCLUDE_NAVEX_NAVIGATION_GOALCALCULATOR_H_ */
