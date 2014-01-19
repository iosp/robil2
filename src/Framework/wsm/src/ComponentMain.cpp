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


void ComponentMain::handleExecuteWorkSequenceCommand( robil2_msgs::String msg)
{

}

void ComponentMain::handlePosAttVel( robil2_msgs::String msg)
{

}

void ComponentMain::handleWiresLengths( robil2_msgs::String msg)
{

}

void ComponentMain::publishBladePosition(robil2_msgs::String &msg)
{
  _roscomm->publishBladePosition(msg);
}

void ComponentMain::publishTrottleEffort(robil2_msgs::String &msg)
{
  _roscomm->publishTrottleEffort(msg);
}
void ComponentMain::publishSteeringEffort(robil2_msgs::String &msg)
{
  _roscomm->publishSteeringEffort(msg);
}

void ComponentMain::publishJointsEffort(robil2_msgs::String &msg)
{
  _roscomm->publishJointsEffort(msg);
}
