/*
 * Filename: RosTopicMapDataSource.cpp
 *   Author: Igor Makhtes
 *     Date: Jun 10, 2015
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


#include <navex/costmap/datasource/RosTopicMapDataSource.h>


RosTopicMapDataSource::RosTopicMapDataSource(const string& topicName,
		bool firstMapOnly)
	: topicName_(topicName), firstMapOnly_(firstMapOnly), mapReceived_(false) {
	ros::NodeHandle node;

	mapSubscriber_ = node.subscribe(
			topicName, 100, &RosTopicMapDataSource::mapCallback, this);
}

RosTopicMapDataSource::~RosTopicMapDataSource() {
}

void RosTopicMapDataSource::mapCallback(
		const nav_msgs::OccupancyGrid::Ptr& map) {

	ROS_INFO("Map received via ROS topic");

	if (firstMapOnly_ && mapReceived_)
		return; // First map already in use

	mapReceived_ = true;

	/// Resize costmap
	CostMapDataSourceBase::createOccupancyGrid(
			map->info.width * map->info.resolution,
			map->info.height * map->info.resolution,
			map->info.origin.position.x,
			map->info.origin.position.y,
			map->info.resolution,
			map->header.frame_id);

	CostMapDataContainer points(true);

	/// Fill data
	for (uint32_t y = 0; y < map->info.height; ++y) {
		for (uint32_t x = 0; x < map->info.width; ++x) {
			const int8_t& cellValue = map->data[y * map->info.width + x];
			if (cellValue > 0) {

				points.push_back(PointData(x, y, 0, CostMapCell::CELL_BLOCKED));
			}
		}
	}

	updatePoints(points);

}
