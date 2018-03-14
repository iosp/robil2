/*
 * Fuzzy.h
 *
 *  Created on: Jan 15, 2017
 *      Author: assaf
 */

#ifndef NAVEX_NAVEX_NAVIGATION_INCLUDE_NAVEX_NAVIGATION_FUZZY_H_
#define NAVEX_NAVEX_NAVIGATION_INCLUDE_NAVEX_NAVIGATION_FUZZY_H_

#include <iostream>
#include <vector>
#include <cmath>
#include <navex/trajectory/Trajectory.h>
#include <navex/costmap/CostMap.h>
#include <navex/trajectory/simulator/models/SkidSteerModel.h>
#include <assert.h>
#include <fstream>

#define NEED_REVIEW {bool THIS_CODE_NEED_REVIEW_AND_FIXING=false; assert(THIS_CODE_NEED_REVIEW_AND_FIXING);}

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

	FuzzyValue operator not()const{ return FuzzyValue(1-v); }

	FuzzyValue operator -()const{ return FuzzyValue(1-v); }
	FuzzyValue operator*(const FuzzyValue& v2)const{ return FuzzyValue(v*v2.v); }
	FuzzyValue operator/(const FuzzyValue& v2)const{ return FuzzyValue(v/v2.v); }
	FuzzyValue operator+(const FuzzyValue& v2)const{ return FuzzyValue(v+v2.v); }
	FuzzyValue operator-(const FuzzyValue& v2)const{ return FuzzyValue(v-v2.v); }

	FuzzyValue operator*(const double& v2)const{ return FuzzyValue(v*v2); }
	FuzzyValue operator/(const double& v2)const{ return FuzzyValue(v/v2); }
	FuzzyValue operator+(const double& v2)const{ return FuzzyValue(v+v2); }
	FuzzyValue operator-(const double& v2)const{ return FuzzyValue(v-v2); }
};

inline FuzzyValue very(const FuzzyValue& f, double level){ return FuzzyValue(pow(f.v, level)); }
inline FuzzyValue very(const FuzzyValue& f){ return FuzzyValue(f.v*f.v); }
inline FuzzyValue extremely(const FuzzyValue& f){ return FuzzyValue(pow(f.v,8)); }
inline FuzzyValue quite(const FuzzyValue& f){ return FuzzyValue(sqrt(f.v)); }
inline FuzzyValue quite(const FuzzyValue& f, double level){ return FuzzyValue(pow(f.v, 1./level)); }
inline FuzzyValue priority(const FuzzyValue& f, double level){ return FuzzyValue(f.v * level); }

inline double ln(double v){ return log(v)/log(M_E); }
inline double sigmoid(double x){ return 1/(1+exp(-x)); }
inline double logistic(double x){ return 1/sqrt(1+x*x); }
class gaussean{
	double m,s,D,E;
public:
	gaussean(double m, double s):m(m),s(s),D(2*s*s),E(sqrt(D*M_PI)){}
	double operator()(double x)const{ double n = x-m; return exp(-(n*n)/D)/E; }
	double inv(double y)const{ return m+sqrt(-D*ln(y*E)); }
};
class cdf{
	double m,s;
public:
	cdf(double m, double s):m(m),s(s){}
	double operator()(double x)const{ double n = x-m, k=2*sqrt(s); return (1+erf(n/k))*0.5; }
};
class norm_gaussean{
	double m,s,b;
public:
	norm_gaussean(double m, double s):m(m),s(s),b(gaussean(m,s)(m)){}
	double operator()(double x)const{ return gaussean(m,s)(x)/b; }
	double inv(double y)const{ return gaussean(m,s).inv(y*b); }
};

#define y (Y.v)

class FSet_ValidTrajectory
{
public:
	double operator()(const FuzzyValue& Y)const{ return y; }
	FuzzyValue operator()(const double& x)const{ return FuzzyValue(x); }
};

class FS_NormFar
{
public:
	double n;
	FS_NormFar(double _n=1):n(_n){};
	double operator()(const FuzzyValue& Y)const{ return log(y)/log(n); }
	FuzzyValue operator()(const double& x)const{ return FuzzyValue(pow(x,n)); }
};
class FS_NormClose
{
public:
	double n;
	FS_NormClose(double _n=1):n(_n){};
	double operator()(const FuzzyValue& Y)const{ return 1-(log(y)/log(n)); }
	FuzzyValue operator()(const double& x)const{ return FuzzyValue(pow(1-x,n)); }
};

class FS_Straight
{
public:
	double n; double s,p,m;
	FS_Straight(double _n, double _s, double _p, double _m):n(_n), s(_s), p(_p), m(_m){};
	double operator()(const FuzzyValue& v)const
	{
		NEED_REVIEW
		return norm_gaussean(0, s).inv(v);
	}
	FuzzyValue operator()(const double& v)const
	{
		return FuzzyValue( ::pow( norm_gaussean(0, s)(v), p)*(1-m)+m );
	}
};

class FS_Safe
{
public:
	double cell_value;
	double min_speed, max_speed;
	double m,s, nm, ns;
	double blocked_value;

	FS_Safe(double cv, double mis, double mas, double blocked=1E-100)
	: cell_value(cv), min_speed(mis), max_speed(mas)
	, m(max_speed*(1-cell_value)), s(max_speed*0.75*(1-cell_value))
	, nm(min_speed*(1-cell_value)), ns(min_speed*0.75*(1-cell_value))
	, blocked_value(blocked)
	{

	}

	double operator()(const FuzzyValue& Y)const
	{
		NEED_REVIEW
		if( y>(1.-1E-5) ) return max_speed;
		double F = norm_gaussean(m,s)(m);
		return m + fabs(m - norm_gaussean(m,s).inv(y));
	}

	FuzzyValue operator()(const double& x)const
	{
		if(cell_value>0.9) return blocked_value;
		if(x<nm) return norm_gaussean(nm,ns)(x);
		if(x<m) return FuzzyValue(1.0);
		return norm_gaussean(m,s)(x);
	}
};

class FS_Backward
{
public:

	FS_Backward(){}

	double operator()(const FuzzyValue& Y)const
	{
		NEED_REVIEW
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

	double operator()(const FuzzyValue& Y)const
	{
		return max_speed * y;
	}

	FuzzyValue operator()(const double& x)const
	{
		return FuzzyValue(fabs(x)/max_speed);
	}
};

class FS_Freeway
{
public:

	FS_Freeway(){}

	double operator()(const FuzzyValue& Y)const
	{
		return y;
	}

	FuzzyValue operator()(const double& x)const
	{
		return FuzzyValue(x);
	}
};

class FS_Recovery
{
public:

	FS_Recovery(){}

	double operator()(const FuzzyValue& Y)const
	{
		return 1-y;
	}

	FuzzyValue operator()(const double& x)const
	{
		return 1-x;
	}
};


#undef y

class FuzzyTrajectory
{
private:
	Trajectory::Ptr trajectory_;

	tf::Vector3 get_last_point(const CostMap & map)const
	{
		tf::Vector3 currentPose;
		const nav_msgs::Path::Ptr& path = trajectory_->getPath();
		geometry_msgs::PoseStamped pose = path->poses[path->poses.size()-1];
		pose.header.stamp = ros::Time(0); // To get the latest tf available in the buffer
		pose.header.frame_id = map.getFrameId(); /// TODO frame id must be set by trajectory it self

		tf::pointMsgToTF(pose.pose.position, currentPose);

		return currentPose;
	}

public:
	FuzzyTrajectory(const Trajectory::Ptr & trajectory=Trajectory::Ptr())
	:trajectory_(trajectory)
	{

	}

	virtual ~FuzzyTrajectory(){}

	Trajectory::Ptr trajectory()const
	{
		return trajectory_;
	}

	FuzzyValue is_safe(const CostMap& map, double min_speed, double max_speed)const
	{
		//GET PARAMETERS
		SkidSteerModel * motionModel = (SkidSteerModel*)trajectory_->getMotionModel();
		double speed = motionModel->getLinearVelocity();
		double max_cell_value=0;
		double avg_cell_value=0;
		double avg_cell_value_count=0;
		const nav_msgs::Path::Ptr& path = trajectory_->getPath();

		//CALCULATE STATISTICS DATA (average, min, max , etc)
		for(int i=0;i<path->poses.size();i++)
		{
			geometry_msgs::PoseStamped pose = path->poses[i];
			pose.header.stamp = ros::Time(0); // To get the latest tf available in the buffer
			pose.header.frame_id = map.getFrameId(); /// TODO frame id must be set by trajectory it self
			double cell_value = (map.getCellValue(pose) + 1) / 101.0;
			max_cell_value = fmax(max_cell_value, cell_value);

			double w = (5+cell_value*10);

			avg_cell_value += cell_value*w;
			avg_cell_value_count += w;
		}
		avg_cell_value/=avg_cell_value_count;
		if( max_cell_value > 0.9 ) avg_cell_value = max_cell_value;

		//FUZZIFICATION
		return FS_Safe(avg_cell_value, min_speed, max_speed)(motionModel->getLinearVelocity());
	}

	FuzzyValue is_on_goal(const CostMap& map, const tf::Vector3 & original_goal, const tf::Vector3 robot_point, double max_speed, double sim_time)const
	{
		const double max_travel_distance = max_speed*sim_time*1.1;

		//PREPARE GAOL
		tf::Vector3 goal = original_goal;
		if( robot_point.distance(goal) > max_travel_distance )
		{
			tf::Vector3 ug = goal - robot_point;
			ug = ug / ug.length() * max_travel_distance;
			goal = robot_point + ug;
		}

		//GET TRAJETORY PROPERTIES (speed, points, etc)
		SkidSteerModel * motionModel = (SkidSteerModel*)trajectory_->getMotionModel();

		tf::Vector3 last_point = get_last_point(map);
		double speed_sig = motionModel->getLinearVelocity()<0 ? -1 : 1;
		last_point = tf::Vector3(last_point.x(), last_point.y()*speed_sig, 0);

		//CALCULATE DISTANCE RATE
		double d = 1 + last_point.distance(goal);
		double D = 1 + robot_point.distance(goal) + max_travel_distance;
		double eps = 0;//1E-5;

		double distance_rate = fmin(1, d/(D+eps));

		//FUZZIFICATION
		return FS_NormClose(1)(distance_rate);
	}
	FuzzyValue is_straight(const CostMap & map,
		double conf_fazzy_straight_stdev, double conf_fazzy_straight_pow, double conf_fazzy_straight_min
		)const
	{
		tf::Vector3 last_point = get_last_point(map);
		double angle = atan2(last_point.y(),last_point.x());
		const double to_rad = 0.0174533;
		return FS_Straight(1, conf_fazzy_straight_stdev*to_rad, conf_fazzy_straight_pow, conf_fazzy_straight_min)(angle);
	}

	FuzzyValue is_fast(double max_speed)const
	{
		SkidSteerModel * motionModel = (SkidSteerModel*)trajectory_->getMotionModel();

		return FS_Fast(max_speed)(motionModel->getLinearVelocity());
	}

	FuzzyValue is_stable(const CostMap & map, const FuzzyTrajectory & prev_selected, double max_dif)const
	{
		double D = max_dif;
		double eps = 1E-5;
		double d = get_last_point(map).distance(prev_selected.get_last_point(map));
		double distance_rate = fmin(1, d/(D+eps));
		return FS_NormClose(1)(distance_rate);
	}

	FuzzyValue is_backward()const
	{
		SkidSteerModel * motionModel = (SkidSteerModel*)trajectory_->getMotionModel();
		double speed = motionModel->getLinearVelocity();

		return FS_Backward()(speed);
	}

	FuzzyValue is_recovery(const CostMap& map)const
	{
		SkidSteerModel * motionModel = (SkidSteerModel*)trajectory_->getMotionModel();
		double speed = motionModel->getLinearVelocity();
		double max_cell_value=0;
		double avg_cell_value=0;
		double avg_cell_value_count=0;
		const nav_msgs::Path::Ptr& path = trajectory_->getPath();
		tf::Vector3 currentPose;
		for(int i=0;i<path->poses.size();i++)
		{

			geometry_msgs::PoseStamped pose = path->poses[i];
			pose.header.stamp = ros::Time(0); // To get the latest tf available in the buffer
			pose.header.frame_id = map.getFrameId(); /// TODO frame id must be set by trajectory it self

			double cell_value = (map.getCellValue(pose) + 1) / 101.;
			max_cell_value = fmax(max_cell_value, cell_value);

			double w = (5+cell_value*10);

			avg_cell_value += cell_value*w;
			avg_cell_value_count += w;

		}
		avg_cell_value/=avg_cell_value_count;

		return FS_Recovery()(avg_cell_value);
	}
};


#endif /* NAVEX_NAVEX_NAVIGATION_INCLUDE_NAVEX_NAVIGATION_FUZZY_H_ */
