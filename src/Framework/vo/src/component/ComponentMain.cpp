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


void ComponentMain::handleCamera(std_msgs::String msg)
{
  std::cout<< "VO say:" << msg.data << std::endl;
}

void ComponentMain::handleINS(std_msgs::String msg)
{
  std::cout<< "VO say:" << msg.data << std::endl;
}

void ComponentMain::handleTF(std_msgs::String msg)
{
  std::cout<< "VO say:" << msg.data << std::endl;
}

void ComponentMain::publishPosAttVel(std_msgs::String &msg)
{
  _roscomm->publishPosAttVel(msg);
}
