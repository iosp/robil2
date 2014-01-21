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

void ComponentMain::handleRPPPath(std_msgs::String msg)
{
  std::cout<< "WPD say:" << msg.data << std::endl;
}

void ComponentMain::handlePosAttVel(std_msgs::String msg)
{
  std::cout<< "WPD say:" << msg.data << std::endl;
}

void ComponentMain::publishTrottleEffort(std_msgs::String &msg)
{
  _roscomm->publishTrottleEffort(msg);
}

void ComponentMain::publishSteeringEffort(std_msgs::String &msg)
{
  _roscomm->publishSteeringEffort(msg);
}
