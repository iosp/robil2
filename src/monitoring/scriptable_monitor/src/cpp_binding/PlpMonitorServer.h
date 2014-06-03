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

class PlpMonitorServer {
public:
	PlpMonitorServer(ros::NodeHandle& n);
	virtual ~PlpMonitorServer();

	void on_event(Plp::EVENT,const Plp* plp);
	void on_event_for_repeated(Plp::EVENT,const Plp* plp);

	void start_module(std::string script);
	void stop_module(std::string module_name);
	void pause_module(std::string module_name);
	void resume_module(std::string module_name);

private:
	ros::NodeHandle& node;
	ros::Publisher p_add;
	ros::Publisher p_remove;
	ros::Publisher p_pause;
	ros::Publisher p_resume;
};

#endif /* PLPMONITORSERVER_H_ */
