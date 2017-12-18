/*
 * PathDamping.cpp
 *
 *  Created on: Jan 30, 2017
 *      Author: dan
 */

#include "PathDamping.h"
#include "../path_manager/PathManager.h"

#include <ros/ros.h>
#include <assert.h>
#include <fstream>

#define WRITE_TO_DAMPING_FILE 1
#define WRITE_TO_PATH_FILE 1

namespace{
	inline
	double NOW()
	{
		static boost::posix_time::ptime start_sys_time = boost::posix_time::second_clock::local_time();
		return double((double)(boost::posix_time::second_clock::local_time() - start_sys_time).total_microseconds()/(double)1000000.0);
	}
	inline
	ros::NodeHandle& getNodeHandle()
	{
		static ros::NodeHandle node("~");
		return node;
	}
}

PathDamping::PathDamping() {

}

PathDamping::~PathDamping() {
}


void PathDamping::Parameters::update()
{
	ros::NodeHandle& node = getNodeHandle();
	if (!node.getParamCached("dampening/alpha", alpha)) {
		alpha = 0.4;
	}
	if (!node.getParamCached("dampening/smooth_resolution", smooth_resolution)) {
		smooth_resolution = 0.5;
	}
	result_path_step = 0.3;

	assert( alpha >= 0.0 and alpha <= 1.0 );
	assert( smooth_resolution >= 0.01 and smooth_resolution <= 100.0 );
	assert( result_path_step >= 0.01 and result_path_step <= 100.0 );
}

void PathDamping::smoothing(std::vector<PathPlanAlphaBetaFilter::Point>& resulted_plan)
{
	const double diluted_max_step = 4.0;

	path_manager::Points extended(resulted_plan.size());
	for(size_t i=0;i<resulted_plan.size();i++)
	{
		extended[i] = path_manager::Vec3(resulted_plan[i].x, resulted_plan[i].y, 0);
	}

	path_manager::Points diluted;
	path_manager::dilute(extended, diluted, 0.1);
	path_manager::extend(diluted, diluted_max_step, diluted);

	resulted_plan.resize(diluted.size());
	for(size_t i=0;i<resulted_plan.size();i++)
	{
		resulted_plan[i] = PathPlanAlphaBetaFilter::Point(diluted[i].x(), diluted[i].y());
	}
}

int PathDamping::filter(
		  const MoveBase::Path& last_path_plan,
		  const MoveBase::Path& controller_plan,
		  const geometry_msgs::PoseStamped& goal,
		  const geometry_msgs::PoseStamped& current_position,

		  MoveBase::Path& resulted_plan
  )
{

	PathPlanAlphaBetaFilter::Point rob_point;
	rob_point.x = current_position.pose.position.x;
	rob_point.y = current_position.pose.position.y;

	PathPlanAlphaBetaFilter::Path old_path = cast_path(last_path_plan);
	PathPlanAlphaBetaFilter::Path current_path = cast_path(controller_plan);

	parameters.update();

#if WRITE_TO_DAMPING_FILE == 1
	static std::ofstream dampening_file("/tmp/dampening.log");

	dampening_file<< NOW() << " :  starting calculation: \n";
	dampening_file<< "\t  rob_point: " << rob_point << "\n";
	dampening_file<< "\t  old_path: " << old_path << "\n";
	dampening_file<< "\t  current_path: " << current_path << "\n";
	dampening_file<< "\t  dampening_alpha: " << parameters.alpha << "\n";
	dampening_file<< "\t  dampening_smooth_resolution: " << parameters.smooth_resolution << "\n";
	dampening_file.flush();
#endif//WRITE_TO_DAMPING_FILE == 1

	int error=0;
	std::vector<PathPlanAlphaBetaFilter::Point> dampening_plan =
			dampening_filter.update(
					error,
					rob_point,
					old_path,
					current_path,
					parameters.alpha,
					parameters.result_path_step, //0.3
					parameters.smooth_resolution
			);

	smoothing(dampening_plan);

#if WRITE_TO_DAMPING_FILE == 1
	dampening_file<< "\t end: " << NOW();
	if(error){ dampening_file<<" with error "<<error; }else{ dampening_file << " OK"; }
	dampening_file<<"\n";
	dampening_file<< "\t  result path: " << dampening_plan << "\n";

	dampening_file.flush();
#endif//WRITE_TO_DAMPING_FILE == 1

#if WRITE_TO_PATH_FILE == 1
	static std::ofstream file("/tmp/path.log");
	static long path_id=0;

	file << "id = " << (path_id++) << " -------- " << std::endl;
	file << "   prev.  path : "<<std::endl;
	for(size_t i=0;i<old_path.size();i++)
		file <<"      "<<old_path[i].x << ", "<<old_path[i].y << std::endl;
	file << "   new.   path : "<<std::endl;
	for(size_t i=0;i<current_path.size();i++)
		file <<"      "<<current_path[i].x << ", "<<current_path[i].y << std::endl;



	file << "   result path : "<<std::endl;
	for(size_t i=0;i<dampening_plan.size();i++)
		file <<"      "<<dampening_plan[i].x << ", "<<dampening_plan[i].y << std::endl;


	file << "===========	TIMESTAMP FOR RESULT PATH:  " << NOW() <<std::endl;
	file.flush();
#endif//WRITE_TO_PATH_FILE == 1


	//copy the dampening plan into the current path plan
	std_msgs::Header points_header = controller_plan.header;
	resulted_plan.poses.clear();

	for(int i=0; i<dampening_plan.size(); ++i){
		geometry_msgs::PoseStamped point;

		point.pose.position.x = dampening_plan.at(i).x;
		point.pose.position.y = dampening_plan.at(i).y;

		point.header = points_header;

		//assign the orientation according to the next point.
		if(i< dampening_plan.size()-1){
			//this point has a "next point", to calculate angle from.
			geometry_msgs::Point diff_from_next_point;
			diff_from_next_point.x = dampening_plan.at(i+1).x - point.pose.position.x;
			diff_from_next_point.y = dampening_plan.at(i+1).y - point.pose.position.y;

			double yaw = atan2(diff_from_next_point.y, diff_from_next_point.x);
			tf::Quaternion orientation; orientation.setEuler(yaw, 0, 0);
			point.pose.orientation.x =orientation.x();
			point.pose.orientation.y =orientation.y();
			point.pose.orientation.z =orientation.z();
			point.pose.orientation.w =orientation.w();
		} else {
			//this point has no "next point" to calculate angle from. take the goal as angle.
			point.pose.orientation = goal.pose.orientation;
		}

		resulted_plan.poses.push_back(point);
	}

	return 0;
}



