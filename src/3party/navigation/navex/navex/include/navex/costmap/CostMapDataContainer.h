/*
 * Filename: CostMapDataContainer.h
 *   Author: Igor Makhtes
 *     Date: Jun 30, 2015
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

#ifndef INCLUDE_NAVEX_COSTMAP_COSTMAPDATACONTAINER_H_
#define INCLUDE_NAVEX_COSTMAP_COSTMAPDATACONTAINER_H_


#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>


using namespace std;


/**
 * Contains information about a single point
 */
class PointData {

public:

	PointData(double x, double y, double z, signed char value)
		: x(x), y(y), z(z), value(value) {
	}

public:

	/**
	 * Point coordinates
	 */
	double x;
	double y;
	double z;

	/**
	 * Cell value
	 */
	signed char value;

};


/**
 * Vector of points data
 */
class CostMapDataContainer : public vector<PointData> {

public:

	typedef boost::shared_ptr<CostMapDataContainer> Ptr;

public:

	CostMapDataContainer(bool pixelCoordinates = false,
			const string& frameId = "", const ros::Time& stamp = ros::Time(0));

	virtual ~CostMapDataContainer();

public:

	/**
	 * Creates initialized shared_ptr for CostMapDataContainer
	 * @param frameId
	 * @return
	 */
	static Ptr create(bool pixelCoordinates = false,
			const string& frameId = "", const ros::Time& stamp = ros::Time(0));

public:

	/**
	 * Gets frame id of all points
	 * @return
	 */
	inline string getFrameId() const {
		return frameId_;
	}

	/**
	 * Sets frame id of all points
	 * @param frameId
	 */
	inline void setFrameId(const string& frameId) {
		frameId_ = frameId;
	}

	/**
	 * If true, all points used as pixels coordinates
	 * and the frame id is ignored
	 * @return
	 */
	inline bool isPixelCoordinates() const {
		return pixelCoordinates_;
	}

	inline void setPixelCoordinates(bool pixelCoordinates) {
		pixelCoordinates_ = pixelCoordinates;
	}

	/**
	 * Sets time stamp
	 * @param stamp
	 */
	inline void setStamp(const ros::Time& stamp) {
		stamp_ = stamp;
	}

	/**
	 * Gets time stamp
	 * @return
	 */
	inline ros::Time getStamp() const {
		return stamp_;
	}

private:

	/**
	 * Frame id of all points
	 */
	string frameId_;

	/**
	 * If true, all points used as pixels coordinates
	 * and the frame id is ignored
	 */
	bool pixelCoordinates_;

	/**
	 * Time stamp
	 */
	ros::Time stamp_;

};

#endif /* INCLUDE_NAVEX_COSTMAP_COSTMAPDATACONTAINER_H_ */
