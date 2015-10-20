/*
 * PathPlanAlphaBetaFilter.h
 *
 *  Created on: Oct 7, 2015
 *      Author: dan
 */

#ifndef SRC_PATHPLANALPHABETAFILTER_H_
#define SRC_PATHPLANALPHABETAFILTER_H_

#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <vector>
#include <boost/thread.hpp>

class PathPlanAlphaBetaFilter {
public:

	struct Point {
		double x, y;
		Point() :
				x(0), y(0) {
		}
		Point(double x) :
				x(x), y(x) {
		}
		Point(double x, double y) :
				x(x), y(y) {
		}
		Point operator+(const Point& p) const {
			return Point(x + p.x, y + p.y);
		}
		Point operator-(const Point& p) const {
			return Point(x - p.x, y - p.y);
		}
		Point operator*(const Point& p) const {
			return Point(x * p.x, y * p.y);
		}
		Point operator/(const Point& p) const {
			return Point(x / p.x, y / p.y);
		}
		static Point polar(double a, double l) {
			return Point(l * cos(a), l * sin(a));
		}
		double ang() const {
			return atan2(y, x);
		}
		double len() const {
			return hypot(x, y);
		}
	};

	typedef std::vector<Point> Path;

	PathPlanAlphaBetaFilter();
	virtual ~PathPlanAlphaBetaFilter();

	Path update(
			int& error,
			const Point& current_position,
			const Path& old_path,
			const Path& new_path,
			double alpha,
			double result_path_step = 5,
			double smooth_resolution = 5
			);

	/**
	 * Copies a nav_msgs::Path into a PathPlanAlphaBetaFilter::Path
	 */
	static PathPlanAlphaBetaFilter::Path path_from_pose_stamped_vector_path(const std::vector<geometry_msgs::PoseStamped>& path);
}
;

PathPlanAlphaBetaFilter::Path
approx(const PathPlanAlphaBetaFilter::Path& path, double step, boost::posix_time::ptime& time_limit);

double len(const PathPlanAlphaBetaFilter::Path& path, boost::posix_time::ptime& time_limit);

#endif /* SRC_PATHPLANALPHABETAFILTER_H_ */
