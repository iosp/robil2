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

void ComponentMain::handleMap(robil2_msgs::String msg)
{

}
void ComponentMain::handleMissionGlobalPath(robil2_msgs::String msg)
{

}
void ComponentMain::handleIEDPosAtt(robil2_msgs::String msg)
{

}
void ComponentMain::handlePosAttVel(robil2_msgs::String msg)
{

}
void ComponentMain::handleRPPPath(robil2_msgs::String msg)
{

}

void ComponentMain::publishLocalPathPlan(robil2_msgs::String &msg)
{
  _roscomm->publishLocalPathPlan(msg);
}
