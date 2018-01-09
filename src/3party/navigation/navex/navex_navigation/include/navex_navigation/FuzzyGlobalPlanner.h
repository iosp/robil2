/*
 * FuzzyGlobalPlanner.h
 *
 *  Created on: Jan 15, 2017
 *      Author: assaf
 */

#ifndef NAVEX_NAVEX_NAVIGATION_INCLUDE_NAVEX_NAVIGATION_FUZZYGLOBALPLANNER_H_
#define NAVEX_NAVEX_NAVIGATION_INCLUDE_NAVEX_NAVIGATION_FUZZYGLOBALPLANNER_H_

#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>

#include <navex/planner/global/GlobalPlannerBase.h>
#include <navex/path_search/AStar.h>

#include <navex_navigation/NavexPathFinder.h>


class FuzzyGlobalPlanner : public GlobalPlannerBase {

public:

	FuzzyGlobalPlanner();

	virtual ~FuzzyGlobalPlanner();

public:

	virtual bool makePlan(CostMap& costmap, const geometry_msgs::PoseStamped& start,
			const geometry_msgs::PoseStamped& goal, nav_msgs::Path& path) const;

protected:

	/**
	 * Initialization method, called once
	 */
	virtual void initialize();

private:

	PathSearchBase* pathFinder_;

	ros::ServiceServer makePlanService_;

#define USE_DAMPING 1
#if USE_DAMPING == 1
	mutable geometry_msgs::PoseStamped lastRequiredGoal_;
	mutable nav_msgs::Path lastCalucaltedPath_;
#endif

private:

	bool findNonBlockedNearestPointOnViewline(
			const CostMap& costmap,
			const geometry_msgs::PoseStamped& start,
			const geometry_msgs::PoseStamped& goal,

			geometry_msgs::PoseStamped& result) const;

	bool findNonBlockedNearestPoint(
			const CostMap& costmap,
			const geometry_msgs::PoseStamped& start,

			geometry_msgs::PoseStamped& result) const;

	bool findNonBlockedGoal(
			const CostMap& costmap,
			const geometry_msgs::PoseStamped& start,
			const geometry_msgs::PoseStamped& goal,

			geometry_msgs::PoseStamped& result) const;

	bool findNonBlockedStart(
			const CostMap& costmap,
			const geometry_msgs::PoseStamped& start,

			geometry_msgs::PoseStamped& result) const;

};




#endif /* NAVEX_NAVEX_NAVIGATION_INCLUDE_NAVEX_NAVIGATION_FUZZYGLOBALPLANNER_H_ */
