/*
 * ComponentMain.cpp
 *
 *  Created on: Jan 16, 2014
 *      Author: yuval
 */

#include "ComponentMain.h"
#include "../roscomm/RosComm.h"

ComponentMain::ComponentMain(int argc,char** argv) {
	_roscomm=new RosComm(this,argc, argv);
}

ComponentMain::~ComponentMain() {
	// TODO Auto-generated destructor stub
}

void ComponentMain::handleSensor_SICK(std_msgs::String msg)
{
  std::cout<< "SE_R2RS say:" << msg.data << std::endl;
}
void ComponentMain::handleSensor_IBEO(std_msgs::String msg)
{
  std::cout<< "SE_R2RS say:" << msg.data << std::endl;
}
void ComponentMain::handleSensor_CAM_R(std_msgs::String msg)
{
  std::cout<< "SE_R2RS say:" << msg.data << std::endl;
}
void ComponentMain::handleSensor_CAM_L(std_msgs::String msg)
{
  std::cout<< "SE_R2RS say:" << msg.data << std::endl;
}
