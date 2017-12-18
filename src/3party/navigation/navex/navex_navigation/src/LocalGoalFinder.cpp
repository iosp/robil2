/*
 * Filename: LocalGoalFinder.cpp
 *   Author: Igor Makhtes
 *     Date: Jul 14, 2015
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


#include <navex_navigation/LocalGoalFinder.h>


LocalGoalFinder::LocalGoalFinder(const string& robotFrameId)
	: lastClosestPointIndex_(0), robotFrameId_(robotFrameId),
	  goalDistance_(15.0), lastGoal_(0) {

}

LocalGoalFinder::~LocalGoalFinder() {

}

int LocalGoalFinder::findLocalGoal() {

	size_t closestPoseIndexOnPath = findClosestPoint(path_);
	size_t localGoalIndexOnPath = min(closestPoseIndexOnPath + (int)goalDistance_,
			path_.poses.size() - 1);

	lastClosestPointIndex_ = closestPoseIndexOnPath;

	lastGoal_ = localGoalIndexOnPath;

	return localGoalIndexOnPath;
}

size_t LocalGoalFinder::findClosestPoint(const nav_msgs::Path& path) const {

	double minDistance = numeric_limits<double>::max();
	size_t minDistanceIndex = 0;

	// Robot's position is (0,0) in local map frame

	size_t minIndex = max(lastClosestPointIndex_ - 50, 0);
	size_t maxIndex = min(lastClosestPointIndex_ + 50, (int)path.poses.size());

	tf::StampedTransform poseTf;

	try {
		// Find transform from path's frame to costmap's frame

		tfListener_.waitForTransform(path.header.frame_id,
				robotFrameId_, ros::Time(0), ros::Duration(5.0));

		tfListener_.lookupTransform(
				path.header.frame_id, robotFrameId_, ros::Time(0), poseTf);
	} catch (tf::TransformException& e) {
		ROS_ERROR("Failed to find transform of robot's pose");
		return lastClosestPointIndex_;
	}

	tf::Vector3 currentPose = poseTf.getOrigin();
	tf::Vector3 pathPose;

	for (int i = minIndex; i < maxIndex; ++i) {

		const geometry_msgs::PoseStamped& pose = path.poses[i];
		tf::pointMsgToTF(pose.pose.position, pathPose);

		double distance = currentPose.distance(pathPose);

		if (distance < minDistance) {
			minDistance = distance;
			minDistanceIndex = i;
		}
	}

	return minDistanceIndex;
}
