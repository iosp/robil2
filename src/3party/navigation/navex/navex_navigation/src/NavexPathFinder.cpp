/*
 * Filename: NavexPathFinder.cpp
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

#include <navex_navigation/NavexPathFinder.h>
#include <set>
#include "path_manager/PathManager.h"

const CostMapCell::CellType NavexPathFinder::cell_obstacle = CostMapCell::CELL_BLOCKED;
const CostMapCell::CellType NavexPathFinder::cell_free = CostMapCell::CELL_FREE;
const CostMapCell::CellType NavexPathFinder::cell_uknown = CostMapCell::CELL_UNKNOWN;
const CostMapCell::CellType NavexPathFinder::cell_closed = 101;

NavexPathFinder::NavexPathFinder() {
	in_bypass = false;
	min_distance_to_target=0;
	smooth_time_limit_sec = 10;
	see_check_resolution = 3;
	result_resolution = 10;
	limit_of_points_in_path = 100000;
}

NavexPathFinder::~NavexPathFinder() {
}

namespace{
	struct P
	{
		double x,y;
		P():x(0),y(0){}
		P(const P& p):x(p.x),y(p.y){}
		P(const Point2d& p):x(p.x),y(p.y){}
		bool operator<(const P& p2)const
		{
			const P& p1 = *this;
			if(p1.x<p2.x) return true;
			if(p1.x>p2.x) return false;
			return p1.y<p2.y;
		}
	};
}
bool NavexPathFinder::search_nearest_free_cell_BFS(const NavexPathFinder::Context& context, const Point2d& goal, Point2d& result)const
{
	if( not in_obstacle(goal, context.world) )
	{
		result = goal;
		return true;
	}

	std::set<P> visited;
	struct is_visited_{
		std::set<P>& v;
		is_visited_(std::set<P>& v):v(v){}
		bool operator()(const Point2d& EL)const{ return v.find(P(EL))!=v.end(); }
	} is_visited(visited);

	list<Point2d> opened;
	struct queue_{
		std::set<P>& v;
		list<Point2d>& o;
		queue_(std::set<P>& v, list<Point2d>& o):v(v),o(o){}
		void push(const Point2d& p)
		{
			v.insert(P(p));
			o.push_back(p);
		}
		void pop(Point2d& p)
		{
			p = o.front();
			o.pop_front();
		}
		Point2d pop(){ Point2d p; pop(p); return p; }
	} queue(visited, opened);

	queue.push(goal);

	while(opened.empty()==false)
	{
		Point2d p = queue.pop();

		if( not in_obstacle(p, context.world) )
		{
			result = p;
			return true;
		}

		int X[]={ -1, +1,  0,  0 };
		int Y[]={  0,  0, -1, +1 };
		for(int i=0;i<4;i++){
			Point2d nei( p.x+X[i], p.y+Y[i] );
			if( not is_valid(nei, context.world) or is_visited(nei) ) continue;
			if( not in_obstacle(nei, context.world) )
			{
				result = nei;
				return true;
			}
			queue.push(nei);
		}
	}
	return false;
}

bool NavexPathFinder::search_nearest_on_board_cell(const NavexPathFinder::Context& context, const Point2d& start, const Point2d& goal, Point2d& result)const
{
	Point2d delta = start - goal;
	Point2d step = delta *(1./ hypot(delta.x,delta.y));
	int icount = hypot(delta.x, delta.y);
	result = goal;
	ROS_INFO_STREAM("PATH SEARCH: search_nearest_on_board_cell: "
			<<start.x<<","<<start.y<<" -> "<<goal.x<<","<<goal.y<<" => delta = "
			<<delta.x<<","<<delta.y<<"("<<hypot(delta.x,delta.y)<<"), step = "<<step.x<<","<<step.y<<", icount="<<icount);
	for( int i=0;i<icount;i++ )
	{
		if( is_valid(result, context.world) ) return true;
		result = result + step;
	}
	return false;
}

bool NavexPathFinder::search_nearest_free_cell(const NavexPathFinder::Context& context, Point2d& result)const
{
	Point2d start = context.start;
	Point2d goal = context.goal;
	Point2d on_board;
	if(search_nearest_on_board_cell(context, start, goal, on_board))
	{
		ROS_INFO_STREAM("PATH SEARCH: first point on board is "<<(int)on_board.x<<", "<<(int)on_board.y);

		//result = on_board;
		if(search_nearest_free_cell_BFS( context, on_board, result))
		{
			ROS_INFO_STREAM("PATH SEARCH: non-blocked point is "<<(int)result.x<<", "<<(int)result.y);
			return true;
		}
		ROS_ERROR("PATH SEARCH: non-blocked point is not found");
		return false;
	}
	ROS_ERROR("PATH SEARCH: first point on board is not found");
	return false;
}

void NavexPathFinder::move_from_obstacle(cv::Mat& world, double resolution, vector<Point2d>& path) const {

	const double max_distance_between_points = 5;

	cv::Mat costmap;
	world.convertTo(costmap, CV_64F);
	costmap = costmap + 1.;
	costmap = costmap * (1./101.);

//	cv::Mat w; costmap.convertTo(w, CV_8U, 255.);
//	cv::imwrite("/tmp/COSTMAP.png", w);

	path_manager::Gradient g(costmap);
	path_manager::Points _c_path = path_manager::path<cv::Point2d, path_manager::Vec3>( path );
	path_manager::Points resulted =	path_manager::shift_path_by_gradient( _c_path , g);

	path_manager::extend(resulted, max_distance_between_points/resolution, resulted);

	path = path_manager::path<path_manager::Vec3, cv::Point2d>(resulted);
}

bool NavexPathFinder::findPath(const CostMap& costMap,
		const geometry_msgs::PoseStamped& start,
		const geometry_msgs::PoseStamped& goal, nav_msgs::Path& path) const {

	Context context;
	cv::Mat original_world;

//	cv::namedWindow("map", CV_WINDOW_NORMAL);

	context.start = cast(costMap.poseToPixel(start));
	context.goal  = cast(costMap.poseToPixel(goal ));

	ROS_INFO_STREAM("PATH SEARCH: Start: " << context.start);
	ROS_INFO_STREAM("PATH SEARCH: Goal : " << context.goal);

	costMap.getCvMatrix().copyTo(context.world);
	costMap.getCvMatrix().copyTo(original_world);

	bool start_in_obstacle = in_obstacle(context.start, context.world);
	if (start_in_obstacle)
	{
		ROS_INFO_STREAM("PATH SEARCH: START IS UNAVAILABLE ("<<(!is_valid(context.start,context.world)?"OUT OF RANGE":"IN OBSTACLE")<<")");
		Point2d tmp = context.start;
		if(not search_nearest_free_cell_BFS(context,  tmp, context.start))
		{
			ROS_INFO_STREAM("PATH SEARCH: FAILED ON SEARCH NEAREST FOR START");
			return false;
		}
		ROS_INFO("Path finder  Corrected start: (%f, %f)", (float)context.start.x, (float)context.start.y);
	}

	bool goal_in_obstacle = in_obstacle(context.goal, context.world);
	if (goal_in_obstacle)
	{
		ROS_INFO_STREAM("PATH SEARCH: GOAL IS UNAVAILABLE ("<<(!is_valid(context.goal,context.world)?"OUT OF RANGE":"IN OBSTACLE")<<")");
		if(not search_nearest_free_cell(context,  context.goal))
		{
			ROS_INFO_STREAM("PATH SEARCH: FAILED ON SEARCH NEAREST FOR GOAL");
			return false;
		}
		ROS_INFO("PATH SEARCH: Corrected goal: (%f, %f)", (float)context.goal.x, (float)context.goal.y);
	}


	ROS_INFO_STREAM("PATH SEARCH: Final Start: " << P2d(context.start).point);
	ROS_INFO_STREAM("PATH SEARCH: Final Goal : " << P2d(context.goal).point);

	context.pointer.location = context.start;

	do {

		Point2d np;

		if(boost::this_thread::interruption_requested()) return false;

		if (getNextPosition(context, context.pointer.location, np)){
			context.pointer.location = np;
		} else {

//			cv::imshow("map", context.world);
//			cv::waitKey();

			if (context.goal == np) {
				ROS_INFO("PATH SEARCH: extract path ...");
				context.path = extract_path(context);
				ROS_INFO_STREAM("PATH SEARCH: extract path : number of points is "<<context.path.size());

				ROS_INFO("PATH SEARCH: smooth path ...");
				smooth(context, context.path);
				ROS_INFO_STREAM("PATH SEARCH: smooth path : number of points is "<<context.path.size());

				ROS_INFO("PATH SEARCH: repulse path ...");
				move_from_obstacle(original_world, costMap.getOccupancyGrid()->info.resolution, context.path);
				ROS_INFO_STREAM("PATH SEARCH: repulse path : number of points is "<<context.path.size());

				if(boost::this_thread::interruption_requested()) return false;

				// Create nav_msgs::Path message result

				path.poses.clear();
				path.header.frame_id = costMap.getFrameId();
				path.header.stamp = ros::Time::now();

				for (int i = context.path.size() - 1; i >= 0; --i) {
					const Point2d& p = context.path[i];

					geometry_msgs::PoseStamped pose = costMap.pixelToPose(p);
					path.poses.push_back(pose);

					// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
					// REMOVE ME
					// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
					if (path.poses.size() >= limit_of_points_in_path)
					{
						ROS_INFO_STREAM("PATH SEARCH: TIMEOUT (TWO MACH POINTS IN CURRENT PATH)");
						//break;
						return false;
					}
				}

				ROS_INFO_STREAM("PATH SEARCH: SUCCESS");
				return true;
			}

			ROS_INFO_STREAM("PATH SEARCH: FAILURE");
			return false;

		}

	} while(true);

}
