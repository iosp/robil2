/*
 * PCDataDynamicConfigParameterProvider.h
 *
 *  Created on: Jan 11, 2017
 *      Author: assaf
 */

#ifndef SOURCE_DIRECTORY__NAVEX_NAVEX_INCLUDE_NAVEX_COSTMAP_PARAMETERS_PCDATADYNAMICCONFIGPARAMETERPROVIDER_H_
#define SOURCE_DIRECTORY__NAVEX_NAVEX_INCLUDE_NAVEX_COSTMAP_PARAMETERS_PCDATADYNAMICCONFIGPARAMETERPROVIDER_H_


#include <string>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <navex/costmap/parameters/IParametersProvider.h>
#include <navex/CostMapPCDataConfig.h>


using namespace std;


class PCDataDynamicConfigParameterProvider{

public:

	PCDataDynamicConfigParameterProvider(const string& ns = "costmapdata")
		: nodePrivate_("~/" + ns) {

		dynamicConfigServer_ =
				new dynamic_reconfigure::Server<navex::CostMapPCDataConfig>(
						configMutex_, nodePrivate_);

		dynamicConfigServer_->setCallback(
				boost::bind(&PCDataDynamicConfigParameterProvider::dynamicConfigCallback,
						this, _1, _2));

	}

	virtual ~PCDataDynamicConfigParameterProvider() { }

public:

	/**
	 * Gets map width in meters
	 * @return
	 */
	virtual inline std::string getDataFrameId() const {
		return dynamicConfig_.data_frame_id;
	}

	/**
	 * Gets map height in meters
	 * @return
	 */
	virtual inline double getDecayTime() const {
		return dynamicConfig_.decay_time;
	}

	/**
	 * Sets map width in meters
	 * @param mapWidth
	 */
	virtual inline void setDataFrameId(const std::string & data_frame_id) {
		nodePrivate_.setParam("data_frame_id", data_frame_id);
		dynamicConfig_.data_frame_id = data_frame_id;

		dynamicConfigServer_->updateConfig(dynamicConfig_);
	}

	/**
	 * Sets map height in meters
	 * @param mapHeight
	 */
	virtual inline void setDecayTime(double decay_time) {
		nodePrivate_.setParam("decay_time",decay_time);
		dynamicConfig_.decay_time = decay_time;

		dynamicConfigServer_->updateConfig(dynamicConfig_);
	}

private:

	ros::NodeHandle nodePrivate_;

	boost::recursive_mutex configMutex_;

	dynamic_reconfigure::Server<navex::CostMapPCDataConfig>* dynamicConfigServer_;

	navex::CostMapPCDataConfig dynamicConfig_;


private:

	/**
	 * Dynamic reconfigure config update callback
	 * @param config
	 * @param mast
	 */
	void dynamicConfigCallback(navex::CostMapPCDataConfig& config, uint32_t level) {
		configMutex_.lock();

		dynamicConfig_ = config;

		configMutex_.unlock();
	}

};



#endif /* SOURCE_DIRECTORY__NAVEX_NAVEX_INCLUDE_NAVEX_COSTMAP_PARAMETERS_PCDATADYNAMICCONFIGPARAMETERPROVIDER_H_ */
