/*
 * Filename: local_costmap_test_node.cpp
 *   Author: Igor Makhtes
 *     Date: Jun 22, 2015
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

#include <navex/costmap/CostMap.h>
#include <navex/costmap/datasource/LaserScanDataSource.h>
#include <navex/costmap/parameters/RosParametersProvider.h>


int main(int argc, char **argv) {
	ros::init(argc, argv, "local_costmap_test_node");
	ros::NodeHandle node;

	CostMap costMap(new LaserScanDataSource(node, "/base_scan"), new RosParametersProvider());

	ros::Publisher mapPublisher = node.advertise<nav_msgs::OccupancyGrid>(
			"/local_costmap", 1, true);

	ros::Rate rate(10);
	while(ros::ok()) {
		mapPublisher.publish(costMap.getOccupancyGrid());
		rate.sleep();
		ros::spinOnce();
	}
}





