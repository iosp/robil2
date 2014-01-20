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

void ComponentMain::handleBladePosition(std_msgs::String msg)
{
  std::cout<< "MAP say:" << msg.data << std::endl;
}

void ComponentMain::handlePosAttVel(std_msgs::String msg)
{
  std::cout<< "MAP say:" << msg.data << std::endl;
}

void ComponentMain::handleLaser(std_msgs::String msg)
{
  std::cout<< "MAP say:" << msg.data << std::endl;
}

void ComponentMain::handleCamera(std_msgs::String msg)
{
  std::cout<< "MAP say:" << msg.data << std::endl;
}

void ComponentMain::publishMap(std_msgs::String &msg)
{
  _roscomm->publishMap(msg);
}

void ComponentMain::publishMiniMap(std_msgs::String &msg)
{
  _roscomm->publishMiniMap(msg);
}
