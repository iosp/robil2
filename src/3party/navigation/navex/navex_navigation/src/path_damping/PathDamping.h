/*
 * PathDamping.h
 *
 *  Created on: Jan 30, 2017
 *      Author: dan
 */

#ifndef NAVEX_NAVIGATION_SRC_PATH_DAMPING_PATHDAMPING_H_
#define NAVEX_NAVIGATION_SRC_PATH_DAMPING_PATHDAMPING_H_

#include "PathPlanAlphaBetaFilter.h"
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

namespace MoveBase
{
//typedef std::vector<geometry_msgs::PoseStamped> Path;
typedef nav_msgs::Path Path;
}

inline
geometry_msgs::PoseStamped cast_pose(const tf::Stamped<tf::Pose>& global_pose)
{
	geometry_msgs::PoseStamped current_position;
	tf::poseStampedTFToMsg(global_pose, current_position);
	return current_position;
}

inline
tf::Stamped<tf::Pose> cast_pose(const geometry_msgs::PoseStamped& global_pose)
{
	tf::Stamped<tf::Pose> current_position;
	tf::poseStampedMsgToTF(global_pose, current_position);
	return current_position;
}

/**
 * Copies a std::vector<geometry_msgs::PoseStamped> into a PathPlanAlphaBetaFilter::Path
 */
inline
PathPlanAlphaBetaFilter::Path cast_path(const std::vector<geometry_msgs::PoseStamped>& path) {
	PathPlanAlphaBetaFilter::Path answer;
	for (int i = 0; i < path.size(); ++i) {
		PathPlanAlphaBetaFilter::Point cur;
		cur.x = path.at(i).pose.position.x;
		cur.y = path.at(i).pose.position.y;

		answer.push_back(cur);
	}
	return answer;
}

inline
PathPlanAlphaBetaFilter::Path cast_path(const nav_msgs::Path& path) {
	PathPlanAlphaBetaFilter::Path answer;
	for (int i = 0; i < path.poses.size(); ++i) {
		PathPlanAlphaBetaFilter::Point cur;
		cur.x = path.poses.at(i).pose.position.x;
		cur.y = path.poses.at(i).pose.position.y;

		answer.push_back(cur);
	}
	return answer;
}


class PathDamping {
public:
	PathDamping();
	virtual ~PathDamping();

	PathPlanAlphaBetaFilter dampening_filter;

	class Parameters
	{
	public:
		double alpha;
		double smooth_resolution;
		double result_path_step;
		void update();
	};
	Parameters parameters;

	void smoothing(std::vector<PathPlanAlphaBetaFilter::Point>& resulted_plan);


	int filter(
			  const MoveBase::Path& last_path_plan,
			  const MoveBase::Path& controller_plan,
			  const geometry_msgs::PoseStamped& goal,
			  const geometry_msgs::PoseStamped& current_position,

			  MoveBase::Path& resulted_plan
	  );
};

#endif /* NAVEX_NAVIGATION_SRC_PATH_DAMPING_PATHDAMPING_H_ */
