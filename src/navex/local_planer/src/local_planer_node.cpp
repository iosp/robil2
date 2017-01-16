

#include <ros/ros.h>
#include "Configuration.h"
Configuration::Config conf;

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

using namespace std;
using namespace cv;
using namespace boost;
using namespace boost::posix_time;

typedef tf::Vector3 Vec3;

double to_deg = 57.2958;
double to_rad = 0.0174533;

class FuzzyValue
{
public:
	double v;
	FuzzyValue(double d):v(d){}
	FuzzyValue(const FuzzyValue& f):v(f.v){}
	operator double()const{ return v; }
	FuzzyValue& operator =(const FuzzyValue& f){ v=f.v; return *this; }

	FuzzyValue operator or(const FuzzyValue& v2)const{ return FuzzyValue(1-(1-v)*(1-v2.v)); }
	FuzzyValue operator and(const FuzzyValue& v2)const{ return FuzzyValue(v*v2.v); }

//	FuzzyValue operator or(const FuzzyValue& v2)const{ return FuzzyValue(fmax(v,v2.v)); }
//	FuzzyValue operator and(const FuzzyValue& v2)const{ return FuzzyValue(fmin(v,v2.v)); }

	FuzzyValue operator not()const{ return FuzzyValue(1-v); }

	FuzzyValue operator -()const{ return FuzzyValue(1-v); }
	FuzzyValue operator*(const FuzzyValue& v2)const{ return FuzzyValue(v*v2); }
	FuzzyValue operator/(const FuzzyValue& v2)const{ return FuzzyValue(v/v2); }
	FuzzyValue operator+(const FuzzyValue& v2)const{ return FuzzyValue(v+v2); }
	FuzzyValue operator-(const FuzzyValue& v2)const{ return FuzzyValue(v-v2); }
};
FuzzyValue very(const FuzzyValue& f){ return FuzzyValue(f.v*f.v); }
FuzzyValue quite(const FuzzyValue& f){ return FuzzyValue(sqrt(f.v)); }

inline double ln(double v){ return log(v)/log(M_E); }
inline double sigmoid(double x){ return 1/(1+exp(-x)); }
inline double logistic(double x){ return 1/sqrt(1+x*x); }
class gausse{
	double m,s,D,E;
public:
	gausse(double m, double s):m(m),s(s),D(2*s*s),E(sqrt(D*M_PI)){}
	double operator()(double x)const{ double n = x-m; return exp(-(n*n)/D)/E; }
	double inv(double y)const{ return m+sqrt(-D*ln(y*E)); }
};
class cdf{
	double m,s;
public:
	cdf(double m, double s):m(m),s(s){}
	double operator()(double x)const{ double n = x-m, k=2*sqrt(s); return (1+erf(n/k))*0.5; }
};
class norm_gousse{
	double m,s,b;
public:
	norm_gousse(double m, double s):m(m),s(s),b(gausse(m,s)(m)){}
	double operator()(double x)const{ return gausse(m,s)(x)/b; }
	double inv(double y)const{ return gausse(m,s).inv(y*b); }
};

class FSet_ValidTrajectory
{
public:
	double operator()(const FuzzyValue& v)const{ return v.v; }
	FuzzyValue operator()(const double& v)const{ return FuzzyValue(v); }
};

class FS_NormFar
{
public:
	double n;
	FS_NormFar(double _n=1):n(_n){};
	double operator()(const FuzzyValue& v)const{ return log(v.v)/log(n); }
	FuzzyValue operator()(const double& v)const{ return FuzzyValue(pow(v,n)); }
};
class FS_NormClose
{
public:
	double n;
	FS_NormClose(double _n=1):n(_n){};
	double operator()(const FuzzyValue& v)const{ return 1-(log(v.v)/log(n)); }
	FuzzyValue operator()(const double& v)const{ return FuzzyValue(pow(1-v,n)); }
};

class FS_Straight
{
public:
	double n;
	FS_Straight(double _n=1):n(_n){};
	double operator()(const FuzzyValue& v)const{ return norm_gousse(0, M_PI).inv(v); }
	FuzzyValue operator()(const double& v)const{ return norm_gousse(0, M_PI)(v); }
};

class FS_Safe
{
public:
	double cell_value;
	double min_speed, max_speed;
	double m,s, nm, ns;

	FS_Safe(double cv, double mis, double mas)
	: cell_value(cv), min_speed(mis), max_speed(mas)
	, m(max_speed*(1-cell_value)), s(max_speed*0.25*(1-cell_value))
	, nm(min_speed*(1-cell_value)), ns(min_speed*0.25*(1-cell_value))
	{
		cout<<":: "<<m<<", "<<s<<",  max="<<max_speed<<", cv="<<cell_value<<endl;

	}

	double operator()(const FuzzyValue& v)const
	{
		double y = v.v;
		if( y>(1.-1E-5) ) return max_speed;
		double F = norm_gousse(m,s)(m);
		return m + fabs(m - norm_gousse(m,s).inv(y));
	}

	FuzzyValue operator()(const double& x)const
	{
		cout<<"speed = "<<x<<endl;
		if(cell_value>0.9) return 0;
		if(x<nm) return norm_gousse(nm,ns)(x);
		if(x<m) return FuzzyValue(1.0);
		return norm_gousse(m,s)(x);
	}
};

class FS_RecaverySpeed
{
public:

	FS_RecaverySpeed(){}

	double operator()(const FuzzyValue& v)const
	{
		double y = v.v;
		return y;
	}

	FuzzyValue operator()(const double& x)const
	{
		if( x > 0 ) return FuzzyValue(0.0);
		return FuzzyValue(0.5);
	}
};

class FS_Fast
{
public:
	double max_speed;

	FS_Fast(double ms):max_speed(ms){}

	double operator()(const FuzzyValue& v)const
	{
		double y = v.v;
		return max_speed * y;
	}

	FuzzyValue operator()(const double& x)const
	{
		return FuzzyValue(fabs(x)/max_speed);
	}
};


enum CellType
{
	CellType_FREE=0,
	CellType_CLOSE=1,
	CellType_BLOCKED=2,
};
CellType cell_type(double costmap_cell_value)
{
	if(costmap_cell_value < 0.3) return CellType_FREE;
	if(costmap_cell_value > 0.7) return CellType_BLOCKED;
	return CellType_CLOSE;
}

class Obstacle;
class CostMap
{
public:
	cv::Mat_<double> mat;

	double robot;
	double safe;

	CostMap()
	: mat(1000, 1000, 0.0)
	, robot(20)
	, safe(150)
	{
		safe += robot;
	}

	double& at(Vec3 point)
	{
		return mat(cv::Point(col(point), row(point)));
	}
	const double& at(Vec3 point)const
	{
		return mat(cv::Point(col(point), row(point)));
	}
	double& at(cv::Point point)
	{
		return mat(point);
	}
	const double& at(cv::Point point)const
	{
		return mat(point);
	}


	int row(Vec3 point)const{ return point.y()+mat.rows/2; }
	int col(Vec3 point)const{ return point.x()+mat.cols/2; }
	int y(cv::Point point)const{ return point.y-mat.rows/2; }
	int x(cv::Point point)const{ return point.x-mat.cols/2; }
	cv::Point get_point( Vec3 point )const{ return cv::Point(col(point), row(point)); }
	Vec3 get_point( cv::Point point )const{ return Vec3(x(point), y(point), 0); }

	void draw(cv::Mat& board)const
	{
		for(int r=0; r<mat.rows; r++)
		{
			for(int c=0; c<mat.rows; c++)
			{
				CellType cell = cell_type(mat(cv::Point(c,r)));
				switch(cell)
				{
				case CellType_BLOCKED:
					board.at<cv::Vec3b>(cv::Point(c,r)) = cv::Vec3b(10,247,247);
					break;
				case CellType_CLOSE:
					board.at<cv::Vec3b>(cv::Point(c,r)) = cv::Vec3b(7,125, 125);
					break;
				case CellType_FREE:
					board.at<cv::Vec3b>(cv::Point(c,r)) = cv::Vec3b(255,255,255);
					break;
				}
			}
		}
	}

	void update( Obstacle& obs );
};

class Goal
{
public:
	bool is_defined;
	Vec3 location;

	Goal()
	: is_defined(false)
	{

	}

	void draw(const CostMap& map, cv::Mat& board)
	{
		if( is_defined == false ) return;
		Point p = map.get_point(location);
		circle(board, p, 5, cv::Scalar(255,0,255), -1);
		circle(board, p, 10, cv::Scalar(255,0,255), 2);
	}
};



class Trajectory
{
public:
	vector<Vec3> points;
	typedef vector<Vec3>::const_iterator const_iterator;
	typedef vector<Vec3>::iterator iterator;

	double lspeed, aspeed;
	double score;
	bool is_selected;
	bool is_blocked;

	Trajectory()
	: lspeed(0), aspeed(0)
	, score(0.0)
	, is_selected(false)
	, is_blocked(false)
	{

	}
	Trajectory( double lspeed, double aspeed, double time, int num_of_points )
	: lspeed(lspeed), aspeed(aspeed)
	, score(0.0)
	, is_selected(false)
	, is_blocked(false)
	{
		double angle=0;
		Vec3 pose(0,0,0);
		double time_step = time / (double)num_of_points;
		double lspeed_step = lspeed * time_step;
		double aspeed_step = aspeed * time_step;
		const Vec3 AxisZ(0,0,1);
		for(int i=0;i<num_of_points;i++)
		{
			Vec3 v(lspeed_step, 0 , 0);
			v = v.rotate(AxisZ, angle);
			pose += v;
			angle+= aspeed_step;
			points.push_back(pose);
		}
	}

	iterator begin(){ return points.begin(); }
	const_iterator begin()const{ return points.begin(); }
	iterator end(){ return points.end(); }
	const_iterator end()const{ return points.end(); }

	void update_score(const CostMap& map)
	{
		score = 0;
	}

	double distance(const Trajectory& other)const
	{
		return points.back().distance(other.points.back());
	}

	cv::Scalar score_to_color()const
	{
		if(is_selected)
			return cv::Scalar(0, 0, 255);
		else
			return cv::Scalar(255, 0, 0);
	}

	void draw(const CostMap& map, cv::Mat& board)const
	{
		cv::Scalar color = score_to_color();
		for(vector<Vec3>::const_iterator i=points.begin(); i!=points.end(); i++)
		{
			const Vec3& vec = *i;
			int c = map.col(vec), r = map.row(vec);
			circle(board, cv::Point(c, r), 1, color, 1, -1);
			if(i!=points.begin())
			{
				vector<Vec3>::const_iterator j = i-1;
				const Vec3& vec = *j;
				int cc = map.col(vec), rr = map.row(vec);
				line(board, cv::Point(c, r), cv::Point(cc, rr), color, 1);
			}
		}
	}

	FuzzyValue is_safe(const CostMap& map, double min_speed, double max_speed)const
	{
		double max_cell_value=0;
		for( vector<Vec3>::const_iterator i=points.begin(); i!=points.end(); i++)
		{
			Vec3 const& p = *i;
			double cell_value = map.at(p);
			max_cell_value = fmax(max_cell_value, cell_value);
		}
		std::cout<<"mcv = "<<max_cell_value<<endl;
		return FS_Safe(max_cell_value, min_speed, max_speed)(lspeed);
	}
	FuzzyValue is_on_goal(const Goal& goal)const
	{
		double speed_sig = lspeed<0 ? -1 : 1;
		Vec3 last_point = points.back();
		last_point = Vec3(last_point.x(), last_point.y()*speed_sig, 0);
		const Vec3& robot_point= points.front();
		double d = last_point.distance(goal.location);
		double D = robot_point.distance(goal.location);
		double eps = 1E-5;
		double distance_rate = fmin(1, d/(D+eps));
		return FS_NormClose(1)(distance_rate);
	}
	FuzzyValue is_straight()const
	{
		const Vec3& last_point = points.back();
		double angle = atan2(last_point.y(),last_point.x());
		return FS_Straight(1)(angle);
	}
	FuzzyValue is_fast(double max_speed)const
	{
		return FS_Fast(max_speed)(lspeed);
	}
	FuzzyValue is_stable(const Trajectory& prev_selected, double max_dif)const
	{
		double D = max_dif;
		double eps = 1E-5;
		double d = distance(prev_selected);
		double distance_rate = fmin(1, d/(D+eps));
		return FS_NormClose(1)(distance_rate);
	}
	FuzzyValue is_recavery()const
	{
		return FS_RecaverySpeed()(lspeed);
	}

};


class Planner
{
public:
	vector<Trajectory> trajectories;
	double min_speed, max_speed, sim_time;

	Planner(
		double min_speed, double max_speed, double speed_steep,
		double min_angle, double max_angle, double angle_steep,
		double sim_time, double granularity
	)
	: max_speed(max_speed)
	, min_speed(min_speed)
	, sim_time(sim_time)
	{
		double speed;
		int points = sim_time / granularity;
		for( speed=min_speed; speed<=max_speed; speed+=speed_steep )
		{
			iterate_angles(speed, min_angle, max_angle, angle_steep, sim_time, points);
		}
		if( speed < max_speed - speed_steep*0.5 )
		{
			iterate_angles(max_speed, min_angle, max_angle, angle_steep, sim_time, points);
		}
	}

	void iterate_angles(double speed, double min_angle, double max_angle, double angle_steep, double sim_time, int points)
	{
		double angle;
		double max_abs_angle = fmax(fabs(max_angle),fabs(min_angle));
		for( angle = 0; angle<=max_abs_angle; angle+=angle_steep )
		{
			if( angle <= max_angle )
			{
				create_trajectory(speed, angle, sim_time, points);
			}
			if( min_angle <= -angle  and angle > 0)
			{
				create_trajectory(speed, -angle, sim_time, points);
			}
		}
		if( angle < max_abs_angle - angle_steep*0.5 )
		{
			angle = max_abs_angle;
			if( angle <= max_angle )
			{
				create_trajectory(speed, angle, sim_time, points);
			}
			if( min_angle <= -angle  and angle > 0)
			{
				create_trajectory(speed, -angle, sim_time, points);
			}
		}
	}

	void create_trajectory( double speed, double angle, double sim_time, double points)
	{
		Trajectory tr(speed, angle/sim_time, sim_time, points);
		trajectories.push_back(tr);
	}

	void draw(const CostMap& map, cv::Mat& board)const
	{
		vector<Trajectory>::const_iterator selected=trajectories.end();
		for(vector<Trajectory>::const_iterator i=trajectories.begin(); i!=trajectories.end(); i++)
		{
			i->draw(map, board);
			if(i->is_selected) selected = i;
		}
		if( selected!=trajectories.end() )
		{
			selected->draw(map, board);
		}
	}

	vector<Trajectory>::const_iterator search_selected()const
	{
		vector<Trajectory>::const_iterator selected=trajectories.end();
		for(vector<Trajectory>::const_iterator i=trajectories.begin(); i!=trajectories.end(); i++)
		{
			if(i->is_selected) selected = i;
		}
		return selected;
	}

	void match(const CostMap& map, const Goal& goal)
	{
		if(goal.is_defined==false) return;

		vector<Trajectory>::const_iterator prev_selected = search_selected();
		double max_dif = sim_time*(max_speed-min_speed);

		vector<Trajectory>::iterator best=trajectories.end();
		double best_valid_score = -1;
		for(vector<Trajectory>::iterator i=trajectories.begin(); i!=trajectories.end(); i++)
		{
			Trajectory& trajectory = *i;
			trajectory.is_selected = false;

			FuzzyValue safe = trajectory.is_safe(map, min_speed, max_speed);
			FuzzyValue fast = trajectory.is_fast(max_speed);
			FuzzyValue on_goal = trajectory.is_on_goal(goal);
			FuzzyValue straight = trajectory.is_straight();
			FuzzyValue recavery = trajectory.is_recavery();
			FuzzyValue stable = prev_selected==trajectories.end() ?  FuzzyValue(1)  : trajectory.is_stable(*prev_selected, max_dif);

			FuzzyValue valid = very(safe) and quite(fast) and on_goal and quite(straight) and quite(stable) and not recavery;


			cout<<"S: "<<valid<<" = "<<very(safe)<<" and "<<quite(fast)<<" and "<<on_goal<<" and "<<quite(straight)<<" and "<<quite(stable)<<" and "<<(not recavery)<<endl;

			FSet_ValidTrajectory vt;
			double valid_score = vt(valid);

			trajectory.score = valid_score;

			if( valid_score > best_valid_score )
			{
				best = i;
				best_valid_score = valid_score;
			}
		}
		if( best!=trajectories.end() )
		{
			best->is_selected = true;
			cout<<"Best is "<<best->lspeed<<", "<<best->aspeed*to_deg<<", score = "<<best->score<<endl;
		}
	}
};

class Obstacle
{
public:
	vector<Vec3> objects;

	void draw(const CostMap& map, cv::Mat& board)
	{
		for(vector<Vec3>::const_iterator i = objects.begin(); i!=objects.end(); i++)
		{
			Point p = map.get_point(*i);
			circle(board, p, 5, cv::Scalar(50,100,200), -1);
		}
	}

	void add( Vec3 p )
	{
		objects.push_back(p);
	}
	void remove( Vec3 p )
	{
		vector<Vec3> tmp;
		for(vector<Vec3>::const_iterator i = objects.begin(); i!=objects.end(); i++)
		{
			if(i->distance(p) > 10) tmp.push_back(*i);
		}
		objects = tmp;
	}
};

void CostMap::update( Obstacle& obs )
{
	rectangle(mat, Point(0,0), Point(mat.cols, mat.rows), Scalar(0.0), -1);

	for(vector<Vec3>::const_iterator i = obs.objects.begin(); i!=obs.objects.end(); i++)
	{
		Point p = get_point(*i);
		circle(mat, p, safe, Scalar(0.5), -1);
	}
	for(vector<Vec3>::const_iterator i = obs.objects.begin(); i!=obs.objects.end(); i++)
	{
		Point p = get_point(*i);
		circle(mat, p, robot, Scalar(1.0), -1);
	}
}

/* ================================================================= */

Goal goal;
CostMap costmap;
Obstacle obstacles;

/* ================================================================= */


void on_mouse(int event, int x, int y, int flags, void* userdata)
{
	static int pressed_mouse = -1;
	//--------------------
	if  ( event == EVENT_MBUTTONDOWN )
	{
		pressed_mouse  = event;
		goal.location = costmap.get_point(cv::Point(x,y));
		goal.is_defined = true;
	}
	if( event == EVENT_LBUTTONDOWN )
	{
		pressed_mouse  = event;
		obstacles.add( costmap.get_point(cv::Point(x,y)) );
	}
	if( event == EVENT_RBUTTONDOWN )
	{
		pressed_mouse  = event;
		obstacles.remove( costmap.get_point(cv::Point(x,y)) );
	}
	//--------------------
	if( event == EVENT_MBUTTONUP )
	{
		pressed_mouse  = -1;
	}
	if( event == EVENT_LBUTTONUP )
	{
		costmap.update( obstacles );
		pressed_mouse  = -1;
	}
	if( event == EVENT_RBUTTONUP )
	{
		costmap.update( obstacles );
		pressed_mouse  = -1;
	}
	//-------------------
	if  ( event == EVENT_MOUSEMOVE )
	{
		event = pressed_mouse;
		if( event == EVENT_MBUTTONDOWN )
		{
			pressed_mouse  = event;
			goal.location = costmap.get_point(cv::Point(x,y));
			goal.is_defined = true;
		}
		if ( event == EVENT_LBUTTONDOWN )
		{
			pressed_mouse  = event;
			obstacles.add( costmap.get_point(cv::Point(x,y)) );
		}
		if( event == EVENT_RBUTTONDOWN )
		{
			pressed_mouse  = event;
			obstacles.remove( costmap.get_point(cv::Point(x,y)) );
		}
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_reconfigure_node");
  ros::NodeHandle node;
  Configuration c(conf);

  namedWindow("Map", CV_WINDOW_NORMAL);
  setMouseCallback("Map", on_mouse, NULL);

  Planner planner(
	-10, 30, 5,
	-60*to_rad, +60*to_rad, 3*to_rad,
	10., 0.1
  );

  ros::Rate rate(30);
  while(ros::ok())
  {
	  planner.match(costmap, goal);


	  Mat board( costmap.mat.rows, costmap.mat.cols, CV_8UC3, cv::Scalar(255,255,255) );
	  costmap.draw(board);
	  obstacles.draw(costmap, board);
	  planner.draw(costmap, board);
	  goal.draw(costmap, board);

	  imshow("Map", board);
	  int k = cv::waitKey(10);
	  if(k>0)
	  {
		  std::cout<<"key = "<<k<<std::endl;
	  }
	  if(k==1048603 or k==27) ros::shutdown();
	  ros::spinOnce();
	  rate.sleep();
  }
  return 0;
}












