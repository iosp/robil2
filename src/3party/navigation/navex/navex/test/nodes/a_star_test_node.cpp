/*
 * Filename: a_star_test_node.cpp
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

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <navex/costmap/CostMap.h>
#include <navex/costmap/datasource/RosTopicMapDataSource.h>
#include <navex/path_search/AStar.h>



ros::Publisher pathPublisher_;
geometry_msgs::PoseStamped start_;
CostMap* costMap_;


void startPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose) {
	geometry_msgs::PoseStamped stampedPose;
	stampedPose.header = pose.header;
	stampedPose.pose.position = pose.pose.pose.position;
	stampedPose.pose.orientation = pose.pose.pose.orientation;
	start_ = stampedPose;

	cv::Point pixel = costMap_->poseToPixel(stampedPose);

	ROS_INFO_STREAM("Starting pose set to:" << stampedPose);
	ROS_INFO_STREAM("Starting pose pixel: " << pixel);
}

void goalPoseCallback(const geometry_msgs::PoseStamped& pose) {
	cv::Point pixel = costMap_->poseToPixel(pose);
	ROS_INFO_STREAM("    Goal pose set to:" << pose);
	ROS_INFO_STREAM("    Goal pose pixel: " << pixel);
	AStar aStar;
	nav_msgs::Path path;

	ROS_INFO("Finding path...");
	if (aStar.findPath(*costMap_, start_, pose, path)) {
		ROS_INFO("Path found!");
		pathPublisher_.publish(path);
		ROS_INFO("Path published, done.");
	} else {
		ROS_ERROR("Failed to find path!");
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "a_star_test_node");
	ros::NodeHandle node;

	costMap_ = new CostMap(new RosTopicMapDataSource("/map", true), 0.25, 0.05, 0.5, 1.0);

	ros::Publisher mapPublisher = node.advertise<nav_msgs::OccupancyGrid>(
			"/costmap_from_topic", 1, true);

	pathPublisher_ = node.advertise<nav_msgs::Path>("/path", 1, true);

	ros::Subscriber startPoseSub = node.subscribe("/initialpose", 1, startPoseCallback);
	ros::Subscriber goalPoseSub = node.subscribe("/goal", 1, goalPoseCallback);

	bool mapPublished = false;

	ros::Rate rate(10.0);
	while(ros::ok()) {

		if (!mapPublished && costMap_->getOccupancyGrid()->data.size() > 0) {
			mapPublisher.publish(costMap_->getOccupancyGrid());
			mapPublished = true;
		}

		rate.sleep();
		ros::spinOnce();
	}
}




