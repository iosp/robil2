/*
 * Filename: AStarGlobalPlanner.h
 *   Author: Igor Makhtes
 *     Date: Jun 23, 2015
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


#ifndef INCLUDE_NAVEX_PLANNER_GLOBAL_ASTARGLOBALPLANNER_H_
#define INCLUDE_NAVEX_PLANNER_GLOBAL_ASTARGLOBALPLANNER_H_


#include <navex/planner/global/GlobalPlannerBase.h>
#include <navex/path_search/AStar.h>


/**
 * A* global planner
 */
class AStarGlobalPlanner: public GlobalPlannerBase {

public:

	AStarGlobalPlanner() { }

	virtual ~AStarGlobalPlanner() { }

public:

	virtual bool makePlan(const CostMap& costmap, const geometry_msgs::PoseStamped& start,
			const geometry_msgs::PoseStamped& goal, nav_msgs::Path& path) const {
		return aStar_.findPath(costmap, start, goal, path);
	}

	/**
	 * Initialization method, called once
	 */
	virtual void initialize() { }

private:

	AStar aStar_;

};

#endif /* INCLUDE_NAVEX_PLANNER_GLOBAL_ASTARGLOBALPLANNER_H_ */
