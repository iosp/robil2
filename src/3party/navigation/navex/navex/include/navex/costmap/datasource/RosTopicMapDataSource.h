/*
 * Filename: RosTopicMapDataSource.h
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


#ifndef INCLUDE_NAVEX_COSTMAP_DATASOURCE_ROSTOPICMAPDATASOURCE_H_
#define INCLUDE_NAVEX_COSTMAP_DATASOURCE_ROSTOPICMAPDATASOURCE_H_


#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <navex/costmap/datasource/CostMapDataSourceBase.h>


/**
 * Receives an occupancy grid via ROS topic
 */
class RosTopicMapDataSource : public CostMapDataSourceBase {

public:

	/**
	 * Constructs map data source
	 * @param topicName map topic name
	 * @param firstMapOnly Whether to use only the first received map
	 */
	RosTopicMapDataSource(const string& topicName, bool firstMapOnly);

	virtual ~RosTopicMapDataSource();

public:

	inline virtual string getName() const {
		return "ROS topic map data source";
	}

private:

	/**
	 * Map topic name
	 */
	string topicName_;

	/**
	 * Whether to use only the first received map
	 */
	bool firstMapOnly_;

	/**
	 * Is map received
	 */
	bool mapReceived_;

	/**
	 * Map subscriber
	 */
	ros::Subscriber mapSubscriber_;

private:

	/**
	 * Map message callback
	 * @param map
	 */
	void mapCallback(const nav_msgs::OccupancyGrid::Ptr& map);

};

#endif /* INCLUDE_NAVEX_COSTMAP_DATASOURCE_ROSTOPICMAPDATASOURCE_H_ */
