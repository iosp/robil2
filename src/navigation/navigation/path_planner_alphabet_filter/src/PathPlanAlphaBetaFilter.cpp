/*
 * PathPlanAlphaBetaFilter.cpp
 *
 *  Created on: Oct 7, 2015
 *      Author: dan
 */

#include "PathPlanAlphaBetaFilter.h"
#include <vector>
#include <iostream>

PathPlanAlphaBetaFilter::PathPlanAlphaBetaFilter() {

}

PathPlanAlphaBetaFilter::~PathPlanAlphaBetaFilter() {
}


namespace
{

	bool search_in_slice(
			const PathPlanAlphaBetaFilter::Point& current_position,
			const PathPlanAlphaBetaFilter::Path& path,
			const double slice,
			const double slice_index,

			PathPlanAlphaBetaFilter::Point& result
	)
	{
		double begin_dist = slice*slice_index;
		double end_dist = slice*(slice_index+1);

		bool found=false;
		int counter = 0;
		for(size_t i=0; i<path.size(); i++)
		{
			double dist = (path[i]-current_position).len();
			if( begin_dist <= dist and dist < end_dist )
			{
				if(not found)
				{
					found = true;
					result = PathPlanAlphaBetaFilter::Point(0,0);
				}
				counter++;
				result = result+path[i];
			}
		}

		if(found)
		{
			result = result / PathPlanAlphaBetaFilter::Point(counter);
		}

		return found;
	}


	int nearest_index(const PathPlanAlphaBetaFilter::Path& path, const PathPlanAlphaBetaFilter::Point point)
	{
		int best=0;
		double best_dist=(path[0]-point).len();
		for(int i=1;i<(int)path.size();i++)
		{
			double dist = (path[i]-point).len();
			if(dist<best_dist)
			{
				best = i;
				best_dist = dist;
			}
		}
		return best;
	}

}

	PathPlanAlphaBetaFilter::Path
		approx( const PathPlanAlphaBetaFilter::Path& path, double step )
	{
		if(path.size()<2) return path;

		PathPlanAlphaBetaFilter::Path res_path;
		typedef PathPlanAlphaBetaFilter::Point Point;

		Point pose( path[0] );
		Point heading = path[1] - path[0];

		res_path.push_back(pose);
		Point goal = path[0];
		int goal_index = 0;

		while(true)
		{

			int ni = goal_index; //nearest_index(path, pose);
			bool is_last_point = ni == path.size()-1;
			if(is_last_point)
			{
				if( (path.back()-pose).len() < step ) break;
				goal = path.back();
			}
			else
			{
				double dist = (goal-pose).len();
				if( dist < step*0.5 )
				{
					while( dist < step*0.5 and goal_index != path.size()-1)
					{
						goal_index++;
						goal = path[goal_index];
						dist = (goal-pose).len();
					}
				}
				else
				{
					goal = path[goal_index];
				}
			}

			Point new_heading = goal - pose;
			Point move = Point::polar(heading.ang()*0.0+new_heading.ang()*1.0, step);
			heading = move;

			pose = pose + move;
			res_path.push_back(pose);
		}

		return res_path;
	}

	double len( const PathPlanAlphaBetaFilter::Path& path )
	{
		double sum=0;
		for(size_t i=1;i<path.size();i++)
		{
			sum += (path[i]-path[i-1]).len();
		}
		return sum;
	}



//PathPlanAlphaBetaFilter::Path PathPlanAlphaBetaFilter::update(
//		const Point& current_position,
//		const Path& _old_path,
//		const Path& _new_path,
//		double alpha,
//		double slice
//)
//{
//	Path old_path = _old_path;
//	Path new_path = _new_path;
//
//	if(old_path.size()==0) old_path = new_path;
//	if(new_path.size()==0) new_path = old_path;
//
//	PathPlanAlphaBetaFilter::Path resulted_path;
//
//	Point op_last_point = old_path.back();
//	Point np_last_point = new_path.back();
//
//	double op_dist = (op_last_point - current_position).len();
//	double np_dist = (np_last_point - current_position).len();
//	double max_dist = fmax(op_dist, np_dist);
//
//	int num_indexes = (int) round(max_dist/slice + 0.5);
//
//	Point op_slice_avg = old_path.front();
//	Point np_slice_avg = new_path.front();
//	for(int i=0;i<num_indexes;i++)
//	{
//		bool f = false;
//		f = search_in_slice(current_position, old_path, slice, i, op_slice_avg) or f;
//		f = search_in_slice(current_position, new_path, slice, i, np_slice_avg) or f;
//
//		if( not f ) continue;
//
//		Point result_point = op_slice_avg*Point(alpha) + np_slice_avg*Point(1.0-alpha);
//		resulted_path.push_back(result_point);
//	}
//
//	return resulted_path;
//}

PathPlanAlphaBetaFilter::Path
	PathPlanAlphaBetaFilter::update(
		const Point& current_position,
		const Path& _old_path,
		const Path& _new_path,
		double alpha,
		double result_path_step,
		double smooth_resolution
)
{
	Path old_path = _old_path;
	Path new_path = _new_path;

	if(old_path.size()==0) old_path = new_path;
	if(new_path.size()==0) new_path = old_path;

	PathPlanAlphaBetaFilter::Path resulted_path;

	double op_len = len(old_path);
	double np_len = len(new_path);
	double min_len = fmin(op_len, np_len);

	if(min_len<5) return old_path;

	int num_of_points = min_len/5.0;

	old_path = approx(old_path, op_len/num_of_points);
	new_path = approx(new_path, np_len/num_of_points);

	for(int i=0;i<num_of_points;i++)
	{
		Point p = old_path[i]*Point(alpha) + new_path[i]*Point(1.0-alpha);
		resulted_path.push_back(p);
	}
	return approx(resulted_path, 30);
}













