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


void ComponentMain::handleMissionStatus(std_msgs::String msg)
{
  std::cout<< "SSM say:" << msg.data << std::endl;
}

void ComponentMain::publishStatusData(std_msgs::String &msg)
{
  _roscomm->publishStatusData(msg);
}
