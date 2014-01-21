/*
 * ComponentMain.cpp
 *
 *  Created on: Jan 16, 2014
 *      Author: yuval
 */

#include "ComponentMain.h"
#include "RosComm.h"

ComponentMain::ComponentMain(int argc,char** argv) {
	_roscomm=new RosComm(this,argc, argv);
}

ComponentMain::~ComponentMain() {
	// TODO Auto-generated destructor stub
}

void ComponentMain::handleTrottleEffort(std_msgs::String msg)
{
  std::cout<< "PLR2U say:" << msg.data << std::endl;
}

void ComponentMain::handleStreeringEffort(std_msgs::String msg)
{
  std::cout<< "PLR2U say:" << msg.data << std::endl;
}

void ComponentMain::handleJointsEffort(std_msgs::String msg)
{

}
