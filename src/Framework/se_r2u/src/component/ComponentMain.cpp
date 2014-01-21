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

void ComponentMain::handleSensor_WIRE(std_msgs::String msg)
{
  std::cout<< "SE_R2U say:" << msg.data << std::endl;
}

void ComponentMain::handleSensor_INSGPS(std_msgs::String msg)
{
  std::cout<< "SE_R2U say:" << msg.data << std::endl;
}
