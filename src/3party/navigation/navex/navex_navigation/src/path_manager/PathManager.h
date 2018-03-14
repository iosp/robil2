/*
 * PathManager.h
 *
 *  Created on: Jan 26, 2017
 *      Author: assaf
 */

#ifndef NAVEX_NAVIGATION_SRC_PATH_MANAGER_PATHMANAGER_H_
#define NAVEX_NAVIGATION_SRC_PATH_MANAGER_PATHMANAGER_H_


#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>

#include <vector>
#include <list>
#include <set>
#include <map>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>

#include <tf/tf.h>



namespace path_manager
{

class PathManager {
public:
	PathManager();
	virtual ~PathManager();
};


typedef tf::Vector3 Vec3;

static double to_deg = 57.2958;
static double to_rad = 0.0174533;

inline double getX(const Vec3& p){return p.x();}
inline double getY(const Vec3& p){return p.y();}
inline double getX(const cv::Point2i& p){return p.x;}
inline double getY(const cv::Point2i& p){return p.y;}
inline double getX(const cv::Point2d& p){return p.x;}
inline double getY(const cv::Point2d& p){return p.y;}

template<class T>
T create_point(double x, double y){ struct REQUIRED_EXPLICIT_CASTING{}; throw REQUIRED_EXPLICIT_CASTING(); }

template < >
inline Vec3 create_point<Vec3>(double x, double y){ return Vec3(x,y,0); }

//template < >
//inline cv::Point2i create_point<cv::Point2i>(double x, double y){ return cv::Point2i(round(x),round(y)); }

template < >
inline cv::Point2d create_point<cv::Point2d>(double x, double y){ return cv::Point2d(x,y); }

template<class TFROM, class TTO>
TTO point(const TFROM& t){ return create_point<TTO>(getX(t),getY(t)); }



template<class P, class R>
std::vector<R> path(const std::vector<P>& p)
{
	std::vector<R> res(p.size());
	for( size_t i=0;i<p.size();i++ )
	{
		res[i] = point<P,R>(p[i]);
	}
	return res;
}

typedef std::vector<Vec3> Points;


struct Gradient
{
	cv::Mat_<double> gX, gY;
	cv::Mat costmap;
	double max_value;

	Gradient(const cv::Mat& costmap);

	cv::Point operator()(cv::Point p , cv::Point2d& gradient, double& costmap_value)const;
};

const Points& create_line(Vec3 start, Vec3 goal, double step, Points& line);

void extend(const Points& source, double step, Points& extended);

void moving_avg(const Points& source, Points& dist, int w);

void smooth(const Points& source, double range, int lookahead, Points& smoothed);
void smooth(const Points& source, double range, int lookahead, int times, Points& smoothed);

bool in_map(const cv::Mat& map, const cv::Point& p);

cv::Point get_gradient( const Gradient& gradient, cv::Point p , cv::Point2d& best, double& dif);

void shift( const Points& _source, Points& dist , const Gradient& gradient, int times);

void dilute(const Points& source, Points& dist, double th);

Points shift_path_by_gradient(const Points& points, const Gradient& gradient);



}//namespace path_manager
#endif /* NAVEX_NAVIGATION_SRC_PATH_MANAGER_PATHMANAGER_H_ */
