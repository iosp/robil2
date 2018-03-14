/*
 * Filename: NavexPathFinder.h
 *   Author: Igor Makhtes
 *     Date: Jun 24, 2015
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2015
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#ifndef INCLUDE_NAVEX_NAVIGATION_NAVEXPATHFINDER_H_
#define INCLUDE_NAVEX_NAVIGATION_NAVEXPATHFINDER_H_

#include <iostream>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <set>
#include <map>
#include <list>
#include <vector>
#include <math.h>
#include <algorithm>
#include <sstream>
#include <fstream>

#include <navex/path_search/PathSearchBase.h>


using namespace std;
using namespace cv;
using namespace boost;
using namespace boost::posix_time;


class NavexPathFinder : public PathSearchBase {

public:

	NavexPathFinder();

	virtual ~NavexPathFinder();

public:

	/**
	 * Finds a path from start to goal
	 * @param costMap The map
	 * @param start The start pose
	 * @param goal The goal pose
	 * @param [out] path The found plan
	 * @return True if path found, false otherwise
	 */
	virtual bool findPath(const CostMap& costMap, const geometry_msgs::PoseStamped& start,
			const geometry_msgs::PoseStamped& goal, nav_msgs::Path& path) const;

private:

	struct P2d {
		Point2i point;

		P2d() { }

		P2d(const Point2d& p) : point((int)p.x, (int)p.y) { }

		bool operator<(const P2d& b) const {
			//return memcmp(&point,&(b.point),sizeof(cv::Point2d))<0;
			if( point.x<b.point.x ) return true;
			if( point.x>b.point.x ) return false;
			return point.y<b.point.y;
		}
	};

	struct Pointer{
		Point2d location;
		double heading;
	};

	struct Context{
		Mat world;
		Pointer pointer;
		Point2d start;
		Point2d goal;
		list<Point2d> open_queue;
		map<P2d,P2d> parent;
		vector<Point2d> path;
	};

private:

	static const CostMapCell::CellType cell_obstacle;
	static const CostMapCell::CellType cell_free;
	static const CostMapCell::CellType cell_uknown;
	static const CostMapCell::CellType cell_closed;

private:

	template<typename _Tp> inline
	const _Tp& cell(const Mat& mat, int i0, int i1) const
	{
	    return ((const _Tp*)(mat.data + mat.step.p[0]*i0))[i1];
	}
	template<typename _Tp> inline
	const _Tp& cell(Mat& mat, int i0, int i1) const
	{
	    return ((_Tp*)(mat.data + mat.step.p[0]*i0))[i1];
	}

	double relaxed(double a) const {
		return atan2(sin(a),cos(a));
	}

	void relax(double& a) const {
		a = atan2(sin(a),cos(a));
	}

	double len(const Point2d& p) const {
		return hypot(p.y, p.x);
	}

	double ang(const Point2d& p) const {
		return atan2(p.y, p.x);
	}

	Point2d polar(double a, double l) const {
		return Point2d(l*cos(a),l*sin(a));
	}

	inline	bool is_valid(const Point2d& p, const Mat& mat) const {
		return p.x>=0 and p.y>=0 and p.x<mat.cols and p.y<mat.rows;
	}

	inline	bool in_obstacle(const Point2d& p, const Mat& mat) const {
		return
				not is_valid(p,mat) or
				cell<CostMapCell::CellType>(mat,p.y,p.x) == cell_obstacle;
	}

	inline	bool is_closed(const Point2d& p, const Mat& mat) const{
		return
				is_valid(p,mat) and
				cell<CostMapCell::CellType>(mat,p.y,p.x)==cell_closed;
	}

	inline	bool is_boundary(const Point2d& p, const Mat& mat) const {
		if( not is_valid(p,mat) ) return false;
		if( in_obstacle(p,mat)) return false;
		int x[]={-1,0,1,1,1,0,-1,-1};
		int y[]={-1,-1,-1,0,1,1,1,0};
		for(int i=0;i<8;i++){
			if( in_obstacle(Point2d(p.x+x[i],p.y+y[i]),mat) ) return true;
		}
		return false;
	}

	#define forImage(M) for(int y=0;y<M.rows;y++)for(int x=0;x<M.cols;x++)

	Vec3b cast(const Scalar& s) const {
		return Vec3b(s[0],s[1],s[2]);
	}

	Scalar cast(const Vec3b& s) const {
		return Scalar(s[0],s[1],s[2]);
	}

	Point cast(const Point2d& p) const {
		return Point(round(p.x),round(p.y));
	}

	inline Point2d cast(const Point& p) const {
		return Point2d(p.x, p.y);
	}

	bool search_nearest_free_cell(const NavexPathFinder::Context& context, Point2d& result)const;
	bool search_nearest_free_cell_BFS(const NavexPathFinder::Context& context, const Point2d& goal, Point2d& result)const;
	bool search_nearest_on_board_cell(const NavexPathFinder::Context& context, const Point2d& start, const Point2d& goal, Point2d& result)const;

private:

	mutable bool in_bypass;
	mutable double min_distance_to_target;
	int smooth_time_limit_sec;
	int see_check_resolution;
	int result_resolution;
	size_t limit_of_points_in_path;

private:

	bool smooth_timeout(boost::posix_time::ptime start_time) const {
		if(boost::this_thread::interruption_requested()) return true;
	    return (boost::posix_time::microsec_clock::local_time()-start_time) > boost::posix_time::seconds(smooth_time_limit_sec);
	}

	bool getNextPosition(Context& context, const Point2d p, Point2d& n, string* error_code = 0) const {

#		define RETURN_FALSE(M) if(error_code){stringstream b; b << "FALSE: "<<M; { *error_code = b.str(); } /*ROS_INFO_STREAM("PATH SEARCH : NEXT: F("<<b.str()<<")");*/} return false
#		define RETURN_TRUE(M)  if(error_code){stringstream b; b << "TRUE : "<<M; { *error_code = b.str(); } /*ROS_INFO_STREAM("PATH SEARCH : NEXT: T("<<b.str()<<")");*/} return true

		int x[]={-1,0,1,1,1,0,-1,-1, -1,1,2,2,1,-1,-2,-2};
		int y[]={-1,-1,-1,0,1,1,1,0, -2,-2,-1,1,2,2,1,-1};
		int K=8;
		int N=16;

	    double current_distance_to_traget = len(context.goal - p);

	    if( current_distance_to_traget < 1)
	    {
	    	context.parent[P2d(context.goal)] = P2d(p);
	    	n = context.goal;
	    	RETURN_FALSE("SUCCESS : current position is same as goal (current_distance_to_traget < 1)");
	    }


	    if(not in_bypass)
	    {
	        min_distance_to_target = current_distance_to_traget;
	    }

	    if(current_distance_to_traget <= min_distance_to_target)
	    {
	    	in_bypass = false;
	    }

		bool _in_obstacle[N];
		bool _is_boundary[N];
		bool _is_free[N];
		bool _is_closed[N];
		context.world.at<CostMapCell::CellType>(p.y,p.x) = cell_closed;
		for(int i=0;i<N;i++){
			Point2d k(p.x+x[i],p.y+y[i]);
			_in_obstacle[i] = in_obstacle(k,context.world);
			if(not _in_obstacle[i]){
				_is_closed[i] = is_closed(k, context.world);
				if(not _is_closed[i]){
					_is_boundary[i] = is_boundary(k, context.world);
					if(not _is_boundary[i]){
						_is_free[i] = true;
					}else{
						_is_free[i] = false;
					}


				}else{
					_is_boundary[i]=_is_free[i]=false;
				}
			}else{
				_is_closed[i]=_is_boundary[i]=_is_free[i]=false;
			}
		}

		int best=-1; double dist=0;
		for(int i=0;i<N;i++){
			Point2d k(p.x+x[i],p.y+y[i]);
			if( k == context.goal ){
//				cout<<"[i] goal detected"<<endl;
				best = i;
				break;
			}
	//		bool if_short_range_then_not_closed = (i<K and not _is_closed[i]);
	//		bool if_long_range_then_not_blocked = ( i>=K and not _in_obstacle[i] and not _is_closed[i]);
			bool if_short_range_then_not_closed = (i<K );
			bool if_long_range_then_not_blocked = ( i>=K and not _in_obstacle[i] );
			if( if_short_range_then_not_closed or if_long_range_then_not_blocked){
				if(best<0 or dist>len(context.goal-k)){
					best = i;
					dist = len(context.goal-k);
				}
			}
		}

		if(best>=0){
	        bool on_road_in_obstacle = _in_obstacle[best] or _is_closed[best] or in_bypass;
			if(best>=K){
				Point2d k(p.x+x[best],p.y+y[best]);
				Point2d d = k-p;
				double head_angle = ang(d);
				Point2d m = polar(head_angle,1);
				Point2d mp = p+m;
				on_road_in_obstacle = on_road_in_obstacle or in_obstacle(mp,context.world);
			}
			if(on_road_in_obstacle){
				//cout<<"[i] the best one is blocked"<<endl;
				for(int i=0;i<N;i++){
					Point2d k(p.x+x[i],p.y+y[i]);
					if(_is_boundary[i]){
						context.open_queue.push_back(k);
						context.world.at<CostMapCell::CellType>(k.y,k.x) = cell_closed;
						context.parent[P2d(k)] = P2d(p);
//						ROS_INFO_STREAM("PATH SEARCH: getNextPos: PARENT["<<P2d(k).point<<"] <= "<<P2d(p).point<<" ("<<k<<","<<p<<")");
						if(i>=K){
							double head_angle = ang(Point2d(p.x+x[i],p.y+y[i])-p);
							Point2d k = polar(head_angle,1);
							context.world.at<CostMapCell::CellType>(k.y,k.x) = cell_closed;
						}
					}
				}
			}else{
				Point2d k(p.x+x[best],p.y+y[best]);
				context.open_queue.push_back(k);
				context.world.at<CostMapCell::CellType>(k.y,k.x) = cell_closed;
				context.parent[P2d(k)] = P2d(p);
//				ROS_INFO_STREAM("PATH SEARCH: getNextPos: PARENT["<<P2d(k).point<<"] <= "<<P2d(p).point<<" ("<<k<<","<<p<<")");

	            //if(false)
	            for(int i=0;i<N;i++){
	                Point2d k(p.x+x[i],p.y+y[i]);
	                if(_is_boundary[i]){
	                    context.open_queue.push_back(k);
	                    context.world.at<CostMapCell::CellType>(k.y,k.x) = cell_closed;
	                    context.parent[P2d(k)] = P2d(p);
//	                    ROS_INFO_STREAM("PATH SEARCH: getNextPos: PARENT["<<P2d(k).point<<"] <= "<<P2d(p).point<<" ("<<k<<","<<p<<")");

	                    if(i>=K){
	                        double head_angle = ang(Point2d(p.x+x[i],p.y+y[i])-p);
	                        Point2d k = polar(head_angle,1);
	                        context.world.at<CostMapCell::CellType>(k.y,k.x) = cell_closed;
	                    }
	                }
	            }


				if(best>=K){
					double head_angle = ang(Point2d(p.x+x[best],p.y+y[best])-p);
					Point2d k = polar(head_angle,1);
					context.world.at<CostMapCell::CellType>(k.y,k.x) = cell_closed;
				}
			}
		}

		if(context.open_queue.empty())
		{
			RETURN_FALSE("???? : open queue is empty");
		}
		else
		{
			list<Point2d>::iterator best=context.open_queue.end(); double dist=0;
			for(list<Point2d>::iterator i=context.open_queue.begin();i!=context.open_queue.end();i++){
				Point2d k(*i);
				if(best==context.open_queue.end() or dist>len(context.goal-k)){
					best = i;
					dist = len(context.goal-k);
				}
			}
			//cout<<"[i] best distance = "<<dist<<endl;
			n = Point2d (*best);
			context.open_queue.erase(best);

			double distance_to_traget = len(context.goal - n);

			if( P2d(context.goal).point == P2d(n).point )
			{
				n = context.goal;
				RETURN_FALSE("SUCCESS: Next point is a last point");
			}
//			else if( distance_to_traget < 1 )
//			{
//				n = context.goal;
//				context.parent[P2d(context.goal)] = P2d(n);
//				RETURN_FALSE("SUCCESS: Next point is close to a last point");
//			}
			else//context.goal!=n
			{
				RETURN_TRUE("SUCCESS : Next point is not a last point");
			}
		}

#	undef RETUR_ERROR
#	undef RETUR_SUCCESS
	}

	vector<Point2d> extract_path(Context& context) const {
//		ROS_INFO_STREAM("PATH SEARCH: extract_path: "<<context.start<<", "<<context.goal);
		vector<Point2d> path;
		Point2d c = context.goal;
		path.push_back(c);

//		ROS_INFO_STREAM("PATH SEARCH: extract_path: PARENT.size() = "<<context.parent.size());
//		for(map<P2d,P2d>::const_iterator i = context.parent.begin(); i!=context.parent.end(); i++)
//		{
//			ROS_INFO_STREAM("PATH SEARCH: extract_path: PARENT["<<i->first.point<<"] => "<<i->second.point);
//		}

		while(c!=context.start){

			P2d key(c);
			map<P2d,P2d>::const_iterator cc = context.parent.find(key);

			if( cc == context.parent.end() )
			{
				ROS_WARN_STREAM("PATH SEARCH: extract_path: " << "Cann't find parent for "<<key.point);
				break;
			}

			//if( path.size() < 50 ) ROS_INFO_STREAM("PATH SEARCH: extract_path: c = "<<key.point<<" -> cc = "<<cc->second.point);

			c = Point2d(cc->second.point.x,cc->second.point.y);
			path.push_back(c);


			// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			// REMOVE ME
			// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			if (path.size() >= limit_of_points_in_path)
			{
				ROS_WARN_STREAM("PATH SEARCH: extract_path: limit of points in path ["<<__FILE__<<":"<<__LINE__<<"]");
				return path;
			}
		}
		//ROS_INFO_STREAM("PATH SEARCH: extract_path: success");
		return path;
	}

	bool is_valid(const vector<Point2d>& path, Context& context) const {
		for(size_t i=0;i<path.size();i++){
			if(in_obstacle(path[i], context.world)) return false;
		}
		return true;
	}

	bool is_valid(Context& context, const vector<Point2d>& path) const {
		for(size_t i=0;i<path.size();i++){
			if(in_obstacle(path[i],context.world)) return false;
		}
		return true;
	}

	vector<Point2d> create_line(Point2d s, Point2d e, double d) const{
		Point2d dir = e-s;
		dir = polar(ang(dir),d);
		vector<Point2d> res; res.push_back(s);
		double dist = len(e-s);
		while( dist >= d ){
			s = s + dir;
			res.push_back(s);
			dist = len(e-s);
		}
		res.push_back(e);
		return res;
	}

	static double fmin( double a, double b) {
		if( a < b ) return a; return b;
	}
	static double fmax( double a, double b) {
		if( a > b ) return a; return b;
	}

	size_t find_next(Context& context, vector<Point2d>& path, long c, boost::posix_time::ptime start_time) const {
		if (path.size() == 0)
			return c + 1;

	    long l = fmin( path.size()-1, c + 10000 );
	    while( l > c+1 ){
	        vector<Point2d> line = create_line(path[c],path[l], see_check_resolution);
	        if(is_valid(line, context)) return l;

	        l-=fmax(10, ((float)l-c)/500.0);

	        if( smooth_timeout(start_time) ){
//	            std::cout<<"[w] not enough time for smoothing"<<endl;
	            return c+1;
	        }
	    }
	    return c+1;
	}
	void smooth(Context& context, vector<Point2d>& path, size_t it=1,
			boost::posix_time::ptime start_time=boost::posix_time::microsec_clock::local_time()) const {

		if(path.size()<2)
		{
			return;
		}

		if(it>=(int)path.size()-1)
		{
			return;
		}

		if( smooth_timeout(start_time) ){
//			std::cout<<"[w] not enough time for smoothing"<<endl;
			return;
		}

		vector<Point2d> reduced_path;
		vector<Point2d> new_path;
		size_t c = 0;
		for(;c<it;c++)  reduced_path.push_back(path[c]);
		while(c<path.size()-1){
			c = find_next(context, path, c, start_time);
			reduced_path.push_back(path[c]);
			if( smooth_timeout(start_time) ) break;
		}
		//cout<<"[i] reduced path size = "<<reduced_path.size()<<endl;

		c = 0;
		for(;c<it;c++)  new_path.push_back(reduced_path[c]);
		for(size_t i=c-1;i<reduced_path.size()-1;i+=1){
			vector<Point2d> line = create_line(reduced_path[i],reduced_path[i+1],result_resolution);
			new_path.insert(new_path.end(), line.begin()+1,line.end());
		}

		//cout<<"[i] new path size = "<<new_path.size()<<endl;
		path = new_path;
		smooth(context, path,it+fmax(result_resolution, path.size()/1000),start_time);
		//smooth(path,it+5,start_time);
	}

	void move_from_obstacle(cv::Mat& world, double resolution, vector<Point2d>& path) const;
};

#endif /* INCLUDE_NAVEX_NAVIGATION_NAVEXPATHFINDER_H_ */
