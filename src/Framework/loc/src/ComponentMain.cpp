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

void ComponentMain::handlePositionUpdate(std_msgs::String msg)
{
  std::cout<< "LOC say:" << msg.data << std::endl;
}

void ComponentMain::handlePosAttVel(std_msgs::String msg)
{
  std::cout<< "LOC say:" << msg.data << std::endl;
}

void ComponentMain::publishPosAttVel(std_msgs::String &msg)
{
  _roscomm->publishPosAttVel(msg);
}
