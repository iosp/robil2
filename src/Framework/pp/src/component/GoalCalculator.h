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

struct dummy_stream:public basic_ostream<char,char_traits<char> >{  };
#define F_ERROR(...) FStreamer(__VA_ARGS__, sERROR)
#define F_WARN(...) FStreamer(__VA_ARGS__, sWARN)
#define F_INFO(...) dummy_stream()//FStreamer(__VA_ARGS__, sINFO)
#define F_DEBUG(...) dummy_stream()//FStreamer(__VA_ARGS__,sDEBUG)

template<class Quaternion>
double quaternion_to_heading(const Quaternion& qq)
{
	tf::Quaternion q;
	q.setW(qq.w);
	q.setX(qq.x);
	q.setY(qq.y);
	q.setZ(qq.z);
	return tf::getYaw(q);
}

enum Severity {sDEBUG, sINFO, sWARN, sERROR};
class FStreamer : public ostream
{
	static bool	active;
	string		_name;
	ostream *	_out;

	void prefix(Severity s) const
	{
		static const string BOLD = "\033[1;";
		static const string LIGHT = "\033[0;";
		string color;

		switch(s)
		{
		case sDEBUG:
			color = "32m";
			break;
		case sINFO:
			color = "36m";
			break;
		case sWARN:
			color = "33m";
			break;
		case sERROR:
			color = "31m";
		}

		if(active)
			*_out << BOLD << color << "[ " << _name << " ]" << LIGHT << color << "\t";
	}

public:
	FStreamer(const string & name, Severity s, ostream * out = &cout):_name(name),_out(out){if(active) prefix(s);}
	~FStreamer(){*_out << "\033[0m";}
	template <class T> ostream& operator<<(const T & t){return active ? (*_out << t) : *_out;}
};

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

	bool translate_goal(const Path & path, Point_2d & goal, const Index wpi, const Point_2d & robot);

	class GoalCalculator
	{
		struct WorldContext
		{
			nav_msgs::OccupancyGrid	_map;
			Point_2d				_robot;
			double					_heading;
		};

		WorldContext			_world;
		boost::shared_ptr<Map>	_cell_map;
		Path					_path;
		Logger					_logger;
		Index					_wpi;

		double					_occupied_level;

		void updateCellMap(bool include_data_update);

	public:
		GoalCalculator(int log_files, double occupied_level):_wpi(0),_occupied_level(occupied_level){_logger.init(log_files);}

		bool updateMap(nav_msgs::OccupancyGrid & new_map, geometry_msgs::PoseWithCovarianceStamped & new_pose);
		void updatePath(nav_msgs::Path & gotten_path);

		bool get(OUTPUT geometry_msgs::PoseStamped & goal, Index & wpi);
	};
}

#endif /* NAVEX_NAVIGATION_INCLUDE_NAVEX_NAVIGATION_GOALCALCULATOR_H_ */
