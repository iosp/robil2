/*
 * states_monitor_aggregator_node.cpp
 *
 *  Created on: Jan 13, 2016
 *      Author: misha
 */
#include "../../include/cognitao/bus/RosStatesMonitorAggregator.h"


using namespace aggregator;

int main(int a, char** aa){
	ros::init(a, aa, "states_monitor_aggregator_node");
	ros::NodeHandle node ("~");
	RosStatesMonitorAggregator states_aggregator (node);
	ros::spin();
	return 0;
}



