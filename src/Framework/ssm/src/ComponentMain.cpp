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


void ComponentMain::handleMissionStatus(robil2_msgs::String msg)
{

}

void ComponentMain::publishStatusData(robil2_msgs::String &msg)
{
  _roscomm->publishStatusData(msg);
}
