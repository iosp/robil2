/*
 * PathPlanAlphaBetaFilter.h
 *
 *  Created on: Oct 7, 2015
 *      Author: dan
 */

#ifndef SRC_PATHPLANALPHABETAFILTER_H_
#define SRC_PATHPLANALPHABETAFILTER_H_

#include <vector>
#include <cmath>
#include <nav_msgs/Path.h>

class PathPlanAlphaBetaFilter {
public:

	struct Point
	{
		double x,y;
		Point():x(0),y(0){}
		Point(double x):x(x),y(x){}
		Point(double x, double y):x(x),y(y){}
		Point operator+(const Point& p)const{ return Point(x+p.x,y+p.y); }
		Point operator-(const Point& p)const{ return Point(x-p.x,y-p.y); }
		Point operator*(const Point& p)const{ return Point(x*p.x,y*p.y); }
		Point operator/(const Point& p)const{ return Point(x/p.x,y/p.y); }
		static Point polar(double a, double l){ return Point(l*cos(a),l*sin(a)); }
		double ang()const{ return atan2(y,x); }
		double len()const{ return hypot(x,y); }
	};

	typedef std::vector<Point> Path;



	PathPlanAlphaBetaFilter();
	virtual ~PathPlanAlphaBetaFilter();


	Path update(
			const Point& current_position,
			const Path& old_path,
			const Path& new_path,
			double alpha,
			double result_path_step = 5,
			double smooth_resolution = 5
	);



};

PathPlanAlphaBetaFilter::Path
	approx( const PathPlanAlphaBetaFilter::Path& path, double step );

double len( const PathPlanAlphaBetaFilter::Path& path );


#endif /* SRC_PATHPLANALPHABETAFILTER_H_ */
