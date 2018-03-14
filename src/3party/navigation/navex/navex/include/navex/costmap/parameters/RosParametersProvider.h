/*
 * Filename: RosParametersProvider.h
 *   Author: Igor Makhtes
 *     Date: Dec 3, 2014
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


#ifndef ROSPARAMETERSPROVIDER_H_
#define ROSPARAMETERSPROVIDER_H_


#include <navex/costmap/parameters/IParametersProvider.h>
#include <ros/ros.h>


/**
 * Cost map parameters provider, that fetches parameters from ROS parameters server
 */
class RosParametersProvider: public IParametersProvider {

public:

	/**
	 * Constructs RosParametersProvider
	 * @param ns
	 */
	inline RosParametersProvider(const string& ns = "costmap")
		: nodePrivate_("~/" + ns) {  }

	virtual ~RosParametersProvider() { }

public:

	/**
	 * Gets map width in meters
	 * @return
	 */
	virtual inline double getMapWidth() const {
		double mapWidth;
		nodePrivate_.param("width", mapWidth, 3.0);
		return mapWidth;
	}

	/**
	 * Gets map height in meters
	 * @return
	 */
	virtual inline double getMapHeight() const {
		double mapHeight;
		nodePrivate_.param("height", mapHeight, 3.0);
		return mapHeight;
	}

	/**
	 * Gets map resolution in p/m
	 * @return
	 */
	virtual inline double getMapResolution() const {
		double resolution;
		nodePrivate_.param("resolution", resolution, 0.025);
		return resolution;
	}

	/**
	 * Gets obstacle inflation radius in meters
	 * @return
	 */
	virtual inline double getInflationRadius() const {
		double inflation;
		nodePrivate_.param("inflation", inflation, 0.05);
		return inflation;
	}

	virtual inline double getInflationSigmma() const {
		double inflation;
		nodePrivate_.param("inflation_gradient_sigmma", inflation, 0.5);
		return inflation;
	}

	virtual inline double getInflationPow() const {
		double inflation;
		nodePrivate_.param("inflation_gradient_pow", inflation, 1.0);
		return inflation;
	}
	/**
	 * Gets robot radius in meters
	 * @return
	 */
	virtual inline double getRobotRadius() const {
		double inflation;
		nodePrivate_.param("robot_radius", inflation, 0.25);
		return inflation;
	}

	/**
	 * Gets the frame id of the cost map
	 * @return
	 */
	virtual inline string getMapFrameId() const {
		string frameId;
		nodePrivate_.param("frame_id", frameId, string("base_link"));
		return frameId;
	}

	/**
	 * Sets map width in meters
	 * @param mapWidth
	 */
	virtual inline void setMapWidth(double mapWidth) {
		nodePrivate_.setParam("width", mapWidth);
	}

	/**
	 * Sets map height in meters
	 * @param mapHeight
	 */
	virtual inline void setMapHeight(double mapHeight) {
		nodePrivate_.setParam("height", mapHeight);
	}

	/**
	 * Sets map resolution in px/m
	 * @param resolution
	 */
	virtual inline void setMapResolution(double resolution) {
		nodePrivate_.setParam("resolution", resolution);
	}

	/**
	 * Sets frame id of the map
	 * @param frameId
	 */
	virtual inline void setFrameId(const string& frameId) {
		nodePrivate_.setParam("frame_id", frameId);
	}

private:

	ros::NodeHandle nodePrivate_;

};

#endif /* ROSPARAMETERSPROVIDER_H_ */
