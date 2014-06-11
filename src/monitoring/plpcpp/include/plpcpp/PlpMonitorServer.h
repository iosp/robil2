/*
 * PlpMonitorServer.h
 *
 *  Created on: Jun 3, 2014
 *      Author: dan
 */

#ifndef PLPMONITORSERVER_H_
#define PLPMONITORSERVER_H_

#include <ros/ros.h>
#include "PlpCpp.h"

namespace plp{

class PlpMonitorServer {
public:
	typedef Module::EVENT EVENT;

	PlpMonitorServer(ros::NodeHandle& n);
	PlpMonitorServer(ros::NodeHandle& n, int _simulate);
	virtual ~PlpMonitorServer();

	void on_event(Module::EVENT,const Module* plp);
	void on_event_for_repeated(Module::EVENT,const Module* plp);

	void start_module(const std::string& script);
	void stop_module(const std::string& module_name);
	void pause_module(const std::string& module_name);
	void resume_module(const std::string& module_name);

private:
	int _simulate;
	ros::NodeHandle& node;
	ros::Publisher p_add;
	ros::Publisher p_remove;
	ros::Publisher p_pause;
	ros::Publisher p_resume;
};


}


#endif /* PLPMONITORSERVER_H_ */
