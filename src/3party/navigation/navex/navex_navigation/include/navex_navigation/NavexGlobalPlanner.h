/*
 * Filename: NavexGlobalPlanner.h
 *   Author: Igor Makhtes
 *     Date: Jun 25, 2015
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

#ifndef SRC_NAVEXGLOBALPLANNER_H_
#define SRC_NAVEXGLOBALPLANNER_H_


#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>

#include <navex/planner/global/GlobalPlannerBase.h>
#include <navex/path_search/AStar.h>

#include <navex_navigation/NavexPathFinder.h>


class NavexGlobalPlanner : public GlobalPlannerBase {

public:

	NavexGlobalPlanner();

	virtual ~NavexGlobalPlanner();

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

	/**
	 * Finds goal that is not blocked and close to the provided goal
	 * @param start
	 * @param goal
	 * @return
	 */
	geometry_msgs::PoseStamped findNonBlockedGoal(
			CostMap& costmap,
			const geometry_msgs::PoseStamped& start,
			const geometry_msgs::PoseStamped& goal) const;

};

#endif /* SRC_NAVEXGLOBALPLANNER_H_ */
