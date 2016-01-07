/*
 * plp.h
 *
 *  Created on: Jun 8, 2014
 *      Author: dan
 */

#ifndef PLP_H_
#define PLP_H_

#include "PlpCpp.h"
#include "PlpMonitorServer.h"
#include "ros/ros.h"

namespace plp{

inline
void init(int argc, char** argv, ros::NodeHandle& node){
	static PlpMonitorServer mon(node,0);
	Module::subscribe(boost::bind(&PlpMonitorServer::on_event,&mon,_1,_2));
}


}


#endif /* PLP_H_ */
