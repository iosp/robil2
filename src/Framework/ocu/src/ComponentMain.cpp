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

void ComponentMain::handlePosAttVel( robil2_msgs::String msg)
{

}
void ComponentMain::handleTeleoperation( robil2_msgs::String msg)
{

}
void ComponentMain::handleStatusData( robil2_msgs::String msg)
{

}
void ComponentMain::handleMissionStatus( robil2_msgs::String msg)
{

}
void ComponentMain::handleMap( robil2_msgs::String msg)
{

}
void ComponentMain::handleLocalPathPlan( robil2_msgs::String msg)
{

}
void ComponentMain::handleIEDDetectionEvent( robil2_msgs::String msg)
{

}
void ComponentMain::handleIEDLocation( robil2_msgs::String msg)
{

}

void ComponentMain::publishPositionUpdate(robil2_msgs::String &msg)
{
  _roscomm->publishPositionUpdate(msg);
}
void ComponentMain::publishMissionPlan(robil2_msgs::String &msg)
{
  _roscomm->publishMissionPlan(msg);
}
void ComponentMain::publishTeleoperation(robil2_msgs::String &msg)
{
  _roscomm->publishTeleoperation(msg);
}
void ComponentMain::publishIEDDetectionEvent(robil2_msgs::String &msg)
{
  _roscomm->publishIEDDetectionEvent(msg);
}
void ComponentMain::publishIEDLocation(robil2_msgs::String &msg)
{
  _roscomm->publishIEDLocation(msg);
}
