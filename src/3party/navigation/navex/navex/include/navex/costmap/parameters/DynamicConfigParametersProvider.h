/*
 * Filename: DynamicConfigParametersProvider.h
 *   Author: Igor Makhtes
 *     Date: Jul 15, 2015
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


#ifndef INCLUDE_NAVEX_COSTMAP_PARAMETERS_DYNAMICCONFIGPARAMETERSPROVIDER_H_
#define INCLUDE_NAVEX_COSTMAP_PARAMETERS_DYNAMICCONFIGPARAMETERSPROVIDER_H_


#include <string>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <navex/costmap/parameters/IParametersProvider.h>
#include <navex/CostMapConfig.h>


using namespace std;


class DynamicConfigParametersProvider: public IParametersProvider {

public:

	DynamicConfigParametersProvider(const string& ns = "costmap")
		: nodePrivate_("~/" + ns) {

		dynamicConfigServer_ =
				new dynamic_reconfigure::Server<navex::CostMapConfig>(
						configMutex_, nodePrivate_);

		dynamicConfigServer_->setCallback(
				boost::bind(&DynamicConfigParametersProvider::dynamicConfigCallback,
						this, _1, _2));

	}

	virtual ~DynamicConfigParametersProvider() { }

public:

	/**
	 * Gets map width in meters
	 * @return
	 */
	virtual inline double getMapWidth() const {
		return dynamicConfig_.width;
	}

	/**
	 * Gets map height in meters
	 * @return
	 */
	virtual inline double getMapHeight() const {
		return dynamicConfig_.height;
	}

	/**
	 * Gets map resolution in p/m
	 * @return
	 */
	virtual inline double getMapResolution() const {
		return dynamicConfig_.resolution;
	}

	/**
	 * Gets obstacle inflation radius in meters
	 * @return
	 */
	virtual inline double getInflationRadius() const {
		return dynamicConfig_.inflation_radius;
	}

	virtual inline double getInflationSigmma() const {
		return dynamicConfig_.inflation_gradient_sig;
	}

	virtual inline double getInflationPow() const {
		return dynamicConfig_.inflation_gradient_pow;
	}

	/**
	 * Gets robot radius in meters
	 * @return
	 */
	virtual inline double getRobotRadius() const {
		return dynamicConfig_.robot_radius;
	}

	/**
	 * Gets the frame id of the cost map
	 * @return
	 */
	virtual inline string getMapFrameId() const {
		return dynamicConfig_.frame_id;
	}

	/**
	 * Sets map width in meters
	 * @param mapWidth
	 */
	virtual inline void setMapWidth(double mapWidth) {
		nodePrivate_.setParam("width", mapWidth);
		dynamicConfig_.width = mapWidth;

		dynamicConfigServer_->updateConfig(dynamicConfig_);
	}

	/**
	 * Sets map height in meters
	 * @param mapHeight
	 */
	virtual inline void setMapHeight(double mapHeight) {
		nodePrivate_.setParam("height", mapHeight);
		dynamicConfig_.height = mapHeight;

		dynamicConfigServer_->updateConfig(dynamicConfig_);
	}

	/**
	 * Sets map resolution in px/m
	 * @param resolution
	 */
	virtual inline void setMapResolution(double resolution) {
		nodePrivate_.setParam("resolution", resolution);
		dynamicConfig_.resolution = resolution;

		dynamicConfigServer_->updateConfig(dynamicConfig_);
	}

	/**
	 * Sets frame id of the map
	 * @param frameId
	 */
	virtual inline void setFrameId(const string& frameId) {
		nodePrivate_.setParam("frame_id", frameId);
		dynamicConfig_.frame_id = frameId;

		dynamicConfigServer_->updateConfig(dynamicConfig_);
	}

private:

	ros::NodeHandle nodePrivate_;

	boost::recursive_mutex configMutex_;

	dynamic_reconfigure::Server<navex::CostMapConfig>* dynamicConfigServer_;

	navex::CostMapConfig dynamicConfig_;


private:

	/**
	 * Dynamic reconfigure config update callback
	 * @param config
	 * @param mast
	 */
	void dynamicConfigCallback(navex::CostMapConfig& config, uint32_t level) {
		configMutex_.lock();

		dynamicConfig_ = config;

		configMutex_.unlock();

		// Notify costmap about parameters change
		this->raiseUpdateParameters();
	}

};

#endif /* INCLUDE_NAVEX_COSTMAP_PARAMETERS_DYNAMICCONFIGPARAMETERSPROVIDER_H_ */
