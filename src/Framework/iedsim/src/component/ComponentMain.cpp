/*
 * ComponentMain.cpp
 *
 *  Created on: Jan 16, 2014
 *      Author: yuval
 */

#include "ComponentMain.h"
#include "../roscomm/RosComm.h"
#include <stdio.h>

ComponentMain::ComponentMain(int argc,char** argv)
{
	_roscomm=new RosComm(this,argc, argv);
}

ComponentMain::~ComponentMain()
{
	// TODO Auto-generated destructor stub
}

void ComponentMain::handleIEDDetectionEvent(std_msgs::String msg)
{
  std::cout<< "IEDSIM say:" << msg.data << std::endl;
}

void ComponentMain::handleIEDLocation(std_msgs::String msg)
{
  std::cout<< "IEDSIM say:" << msg.data << std::endl;
}

void ComponentMain::publishIEDDetectionEvent(std_msgs::String &msg)
{
  _roscomm->publishIEDDetectionEvent(msg);
}

void ComponentMain::publishIEDLocation(std_msgs::String &msg)
{
  _roscomm->publishIEDLocation(msg);
}
