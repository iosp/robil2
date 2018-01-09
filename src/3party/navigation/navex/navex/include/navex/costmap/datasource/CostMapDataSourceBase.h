/*
 * Filename: ICostMapDataSource.h
 *   Author: Igor Makhtes
 *     Date: Nov 28, 2014
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014
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


#ifndef INCLUDE_NAVEX_ICOSTMAPDATASOURCE_H_
#define INCLUDE_NAVEX_ICOSTMAPDATASOURCE_H_


#include <string>

#include <boost/function.hpp>

#include <ros/ros.h>

#include <navex/costmap/CostMapCell.h>
#include <navex/costmap/CostMapDataContainer.h>


using namespace std;


/**
 * Data source base class for CostMap
 */
class CostMapDataSourceBase {

public:

	virtual ~CostMapDataSourceBase() { }

public:

	/**
	 * Name of this data source
	 * @return
	 */
	virtual string getName() const = 0;

	/**
	 * Clears cost map
	 */
	inline void clearMap() const {
		clearMapCallback();
	}

	/**
	 * Publishes points to costmap
	 * @param points
	 */
	inline void updatePoints(const CostMapDataContainer& points) const {
		updatePointsCallback(boost::ref(points));
	}

	/**
	 * Initializes and allocates memory for occupancy grid in CostMap
	 * @param width
	 * @param height
	 * @param resolution
	 * @param frameId
	 */
	inline void createOccupancyGrid(double width, double height,
			double originX, double originY,
			double resolution, const string& frameId) const {
		createOccupancyGridCallback(width, height, resolution, originX, originY, boost::ref(frameId));
	}

public:

	typedef boost::function<void()> ClearMapCallback;

	typedef boost::function<void(const CostMapDataContainer&)> UpdatePointsCallback;

	typedef boost::function<void(double, double,
			double, double ,
			double, const string&)> CreateOccupancyGridCallback;

	ClearMapCallback clearMapCallback;
	UpdatePointsCallback updatePointsCallback;
	CreateOccupancyGridCallback createOccupancyGridCallback;

};

#endif /* INCLUDE_NAVEX_ICOSTMAPDATASOURCE_H_ */
