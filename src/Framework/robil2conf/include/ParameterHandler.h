/*
 * ParameterHandler.h
 *
 *  Created on: Jan 21, 2014
 *      Author: userws1
 */

#ifndef PARAMETERHANDLER_H_
#define PARAMETERHANDLER_H_
#include <ros/ros.h>
#include <string>
#include "ParameterTypes.h"

std::string fetchParam(ros::NodeHandle * nh,std::string compName,std::string neededTopic,std::string type);


#endif /* PARAMETERHANDLER_H_ */
