/*
 * PathManager.cpp
 *
 *  Created on: Jan 26, 2017
 *      Author: assaf
 */

#include "PathManager.h"


using namespace std;
using namespace cv;
using namespace boost;
using namespace boost::posix_time;

namespace path_manager
{


Gradient::Gradient(const cv::Mat& costmap_input)
{
//		cv::Sobel(costmap, gX, CV_64F, 1, 0, 5);
//		cv::Sobel(costmap, gY, CV_64F, 0, 1, 5);
	cv::GaussianBlur(costmap_input, costmap, cv::Size(5,5),0);

	cv::Scharr(costmap, gX, CV_64F, 1, 0);
	cv::Scharr(costmap, gY, CV_64F, 0, 1);

//	cv::GaussianBlur(gX, gX, cv::Size(5,5),0);
//	cv::GaussianBlur(gY, gY, cv::Size(5,5),0);


//	gX = gX * costmap;
//	gY = gY * costmap;

	Mat XX,YY; multiply(gX,gX,XX);multiply(gY,gY,YY);XX = XX+YY;
	double _max, _mix; cv::minMaxLoc(XX,&_mix,&_max);
	max_value = sqrt(_max);
}

Point Gradient::operator()(Point p , Point2d& gradient, double& costmap_value)const
{
	gradient = Point2d(gX.at<double>(p), gY.at<double>(p));
	costmap_value = costmap.at<double>(p);
	return gradient;
}


PathManager::PathManager() {
	// TODO Auto-generated constructor stub

}

PathManager::~PathManager() {
	// TODO Auto-generated destructor stub
}

#define CHECK_IF_INPUT_AND_OUTPUT_ARE_SAME(I, O, RI) \
		Points* __TMP_ARRAY(0); \
		struct __REMOVE_TMP_ARRAY{Points* &t; __REMOVE_TMP_ARRAY(Points* &t):t(t){} ~__REMOVE_TMP_ARRAY(){if(t){delete t;t=0;}} } ___REMOVE_TMP_ARRAY(__TMP_ARRAY);\
		const Points* __TMP_INPUT_PTR = &I;\
		if( __TMP_INPUT_PTR == &O ){ __TMP_ARRAY=new Points(); *__TMP_ARRAY=I; __TMP_INPUT_PTR = __TMP_ARRAY; }\
		const Points& RI = *__TMP_INPUT_PTR;

const Points& create_line(Vec3 start, Vec3 goal, double step, Points& line)
{
	Vec3 delta = goal-start;
	Vec3 d = delta/delta.length() * step;
	Vec3 c = start;
	for( size_t i=0; i<delta.length()/step; i++)
	{
		line.push_back(c);
		c += d;
	}
	if(c.distance(goal)<step*0.3) line.push_back(goal);
	return line;
}

void extend(const Points& source, double step, Points& extended)
{
	CHECK_IF_INPUT_AND_OUTPUT_ARE_SAME(source, extended, points);

	extended.clear();
	if(points.size()<1) return;
	for( Points::const_iterator p=points.begin()+1; p!=points.end(); p++ )
	{
		create_line(*(p-1), *p, step, extended);
	}
	extended.push_back(points.back());
}

void smooth(const Points& source, double range, int lookahead, int times, Points& smoothed)
{
	Points source_;
	Points smoothed_;

	smoothed_ = source;
	for( int i=0;i<times;i++)
	{
		source_ = smoothed_;
		smooth(source_, range, lookahead, smoothed_);
	}

	smoothed = smoothed_;
}

void moving_avg(const Points& source, Points& dist, int w)
{
	if(source.size() < w){ if(&source!=&dist) dist = source; return; }

	CHECK_IF_INPUT_AND_OUTPUT_ARE_SAME(source, dist, points);

	int w05 = w/2;
	dist.resize(points.size());
	for(int i=0;i<w05;i++) dist[i]=points[i];
	for(int i=points.size()-w05;i<points.size();i++) dist[i]=points[i];
	Points::const_iterator l = points.begin();
	Points::const_iterator r = points.begin()+w-1;
	Points::iterator p = dist.begin()+(w05+1);
	Vec3 sum(0,0,0);
	for(Points::const_iterator c=l; c!=r; c++) sum+=*c;

	for(; r!=points.end(); l++, r++, p++)
	{
		sum+=*r;
		*p = sum / (double)w;
		sum-=*l;
	}
}

void smooth(const Points& source, double range, int lookahead, Points& smoothed)
{
	CHECK_IF_INPUT_AND_OUTPUT_ARE_SAME(source, smoothed, extended);

	struct find_in_range_t
	{
		Points::const_iterator operator()(const Vec3& p, double range, const Points& points, Points::const_iterator candidate, int level)const
		{
			if(candidate == points.end()) return candidate;
			if(level==0) if(p.distance(*candidate)<range) return candidate; else return points.end();
			Points::const_iterator result = find_in_range_t()(p, range, points, candidate+1, level-1);
			if(result!=points.end()) return result;
			if(p.distance(*candidate)<range) return candidate; else return points.end();
		}
	} find_in_range;

	if(extended.size()<2) return;

	smoothed.clear();
	Points::const_iterator p = extended.begin();
	while(p!=extended.end())
	{
		smoothed.push_back(*p);
		double c_range = range;
		if( p+1!=extended.end() and p->distance(*(p+1)) > range )
		{
			c_range = p->distance(*(p+1))*1.01;
			p = find_in_range(*p, c_range, extended, p+1, lookahead);
		}
		else
		{
			p = p+1;
		}
	}
}

bool in_map(const Mat& map, const Point& p)
{
	return 0<=p.x and 0<=p.y and p.x<map.cols and p.y<map.rows;
}

Point get_gradient( const Gradient& gradient, Point p , Point2d& best, double& dif)
{
	return gradient(p, best, dif);
}

void shift( const Points& _source, Points& dist , const Gradient& gradient, int times)
{
	dist = _source;
	for(int i=0;i<times;i++)
	{
		Points source = dist;
//		dist.clear();
		for(size_t j =0 ; j < source.size(); j ++)
		{
			Vec3 c, source_value = source[j];
			Point2d g; // gradient
			double v;  // costmap value
			gradient(point<Vec3,Point2d>(source_value), g, v);
			g = g*-1; // we need opposite gradient from higher to lower
			double gm = hypot(g.x,g.y);
			if( gm < 1E-10 ) continue;

			Vec3 ug = point<Point2d,Vec3>(g) / gm;
			dist[j] = source_value + ug*(v*v);
		}
	}
}


inline
double triangle_area( double a, double b, double c )
{
	double s = (a+b+c)/2.;
	double A = sqrt(s*(s-a)*(s-b)*(s-c));
	return A;
}
inline
double triangle_high( double a, double b, double c )
{
	if( c < 0.000001 ) return 0;
	double A = triangle_area(a, b, c);
	double h = 2*A/c;
	return h;
}

namespace{

	 inline double len(const Point2d& p){ return hypot(p.x, p.y); }
	 inline double len(const Vec3& p){ return hypot(p.x(), p.y()); }

}

void dilute(const Points& source, size_t begin, size_t end, Points& dist, double th)
{
	if( end-begin<3 ){ dist = Points(source.begin()+begin, source.begin()+end); return ;}
	const Vec3& first = source[begin];
	const Vec3& last = source[end-1];
	size_t far = 0;
	double bh = 0;
	double c = len(last - first);
	for( size_t i=begin+1; i<end-1; i++ )
	{
		const Vec3& middle = source[i];
		double a = len(middle-first);
		double b = len(last-middle);
		double h = triangle_high(a, b, c);
		if( h > bh )
		{
			far = i;
			bh = h;
		}
	}
	if( bh < th )
	{
		dist.push_back(first);
		dist.push_back(last);
		return;
	}

	Points head, tail;
	dilute( source, begin, far+1, head , th);
	dilute( source, far, end, tail , th);

	dist.insert(dist.end(), head.begin(), head.end());
	dist.insert(dist.end(), tail.begin()+1, tail.end());
}
void dilute(const Points& source, Points& dist, double th)
{
	dilute(source, 0, source.size(), dist, th);
}



Points shift_path_by_gradient(const Points& points, const Gradient& gradient)
{
	const int counter = 20;
	const int extend_step = 2;
	const int smooth_step = 10;
	const int smooth_lookahead = 5;
	const int shift_step = 1;
	const int moving_avg_window = 15;
	//const int diluted_max_step = 5;

	Points extended = points;

	smooth(extended, smooth_step, smooth_lookahead, extended);
	moving_avg(extended, extended, 15);
	extend(extended, extend_step, extended);

	for(int i=0;i<counter;i++)
	{
		shift(extended, extended, gradient, shift_step);

		smooth(extended, smooth_step, smooth_lookahead, extended);
		moving_avg(extended, extended, 15);
		extend(extended, extend_step, extended);
	}

	smooth(extended, smooth_step, smooth_lookahead, extended);
	moving_avg(extended, extended, moving_avg_window);

	Points diluted;
	dilute(extended, diluted, 0.1);
	//extend(diluted, diluted_max_step, diluted);

	return diluted;
}


}//namespace path_manager

