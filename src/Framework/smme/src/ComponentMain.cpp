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

void ComponentMain::handleIEDDetectionEvent(robil2_msgs::String msg)
{

}
void ComponentMain::handleIEDLocation( robil2_msgs::String msg)
{

}
void ComponentMain::handleMissionPlan( robil2_msgs::String msg)
{

}

void ComponentMain::handleStatusData( robil2_msgs::String msg)
{

}

void ComponentMain::publishMissionStatus(robil2_msgs::String &msg)
{
  _roscomm->publishMissionStatus(msg);
}

void ComponentMain::publishMissionGlobalPath(robil2_msgs::String &msg)
{
  _roscomm->publishMissionGlobalPath(msg);
}

void ComponentMain::publishIEDPosAtt(robil2_msgs::String &msg)
{
  _roscomm->publishIEDPosAtt(msg);
}

void ComponentMain::publishExecuteWorkSequenceCommand(robil2_msgs::String &msg)
{
  _roscomm->publishExecuteWorkSequenceCommand(msg);
}
