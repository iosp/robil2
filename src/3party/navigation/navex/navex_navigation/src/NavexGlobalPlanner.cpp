/*
 * Filename: NavexGlobalPlanner.cpp
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


#include <navex_navigation/NavexGlobalPlanner.h>

#if USE_DAMPING == 1
	#include "path_damping/PathDamping.h"
	#include <fstream>
#endif


NavexGlobalPlanner::NavexGlobalPlanner()
	: pathFinder_(new NavexPathFinder()) {
}

NavexGlobalPlanner::~NavexGlobalPlanner() {
	delete pathFinder_;
}

void NavexGlobalPlanner::initialize() {
}

bool NavexGlobalPlanner::makePlan(CostMap& costmap,
		const geometry_msgs::PoseStamped& start,
		const geometry_msgs::PoseStamped& goal, nav_msgs::Path& path) const {

	geometry_msgs::PoseStamped actualGoal = goal;

	if (costmap.getCellValue(start) >= CostMapCell::CELL_BLOCKED) {
		// Starting position is block, try clearing
		ROS_INFO("Starting position is blocked, trying to clear starting location");
		costmap.clearCell(start, costmap.getRobotRadius());
	}

	if (costmap.getCellValue(actualGoal) >= CostMapCell::CELL_BLOCKED) {
		ROS_INFO("Goal is blocked, searching for a nearest not blocked point...");
		actualGoal = findNonBlockedGoal(costmap, start, goal);
	}

	bool pathFound = false;

	try {
		pathFound = pathFinder_->findPath(costmap, start, actualGoal, path);
	} catch (...) {
		ROS_ERROR("Unexpected error occurred while planning");
		return false;
	}

#if USE_DAMPING == 1

	const double min_distance_for_near_objects = 0.01; // 1cm
	const bool is_last_required_goal_near_current_goal =
			cast_pose(goal				).getOrigin().distance(
			cast_pose(lastRequiredGoal_	).getOrigin()
			) < min_distance_for_near_objects;

	static std::ofstream damping_log("/tmp/damping.log");
	if( is_last_required_goal_near_current_goal )
	{
		static PathDamping path_damping;

		damping_log <<"Original path (damping) : "<<endl<< path<<endl;

		path_damping.filter(
				lastCalucaltedPath_,
				path,
				start,
				goal,

				path
		);

		damping_log <<"Damped path : "<<endl<< path<<endl;
	}
	else
	{
		damping_log <<"Original path (no damping) : "<<endl<< path<<endl;
	}

	lastRequiredGoal_ = goal;
	lastCalucaltedPath_ = path;

#endif // USE_DAMPING == 1

	return pathFound;
}

geometry_msgs::PoseStamped NavexGlobalPlanner::findNonBlockedGoal(
		CostMap& costmap,
		const geometry_msgs::PoseStamped& start,
		const geometry_msgs::PoseStamped& goal) const {
	geometry_msgs::PoseStamped actualGoal = goal;

	cv::Point startPoint, goalPoint;

	startPoint = costmap.poseToPixel(start);
	goalPoint = costmap.poseToPixel(goal);

	tf::Vector3 startVec(startPoint.x, startPoint.y, 0);
	tf::Vector3 goalVec(goalPoint.x, goalPoint.y, 0);

	double goalDistance = startVec.distance(goalVec);
	tf::Vector3 stepVector = (startVec - goalVec) / goalDistance; // 1px step

	tf::Vector3 currentPoint = goalVec;

	for (size_t i = 0; i < (int)goalDistance; ++i) {
		currentPoint += stepVector;

		CostMapCell::CellType cellValue =
				costmap.getCellValue(
						(uint32_t)currentPoint.x(), (uint32_t)currentPoint.y());

		if (cellValue < CostMapCell::CELL_BLOCKED) {
			ROS_INFO("Free point found!");

			return costmap.pixelToPose(currentPoint.x(), currentPoint.y());
		}
	}

	// Failed to find
	return start;
}
