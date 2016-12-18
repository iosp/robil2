/*
 * tao_parser_io.h
 *
 *  Created on: Nov 16, 2015
 *      Author: dan
 */

#ifndef TAO_PARSER_ROS_H_
#define TAO_PARSER_ROS_H_


#include "core.h"
#include "io.h"

namespace cognitao{ namespace io{ namespace parser{ namespace xml{ namespace ros{

using namespace core;
using namespace iostream;

string search_ros_package(string ros_pack_name);
string search_ros_parameter(string ros_param_name);

}}}}}


#endif /* TAO_PARSER_ROS_H_ */
