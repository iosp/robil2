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

void ComponentMain::handleTrottleEffort(robil2_msgs::String msg)
{

}

void ComponentMain::handleSteeringEffort(robil2_msgs::String msg)
{

}

void ComponentMain::handleJointsEffort(robil2_msgs::String msg)
{

}

void ComponentMain::handleTeleoperation(robil2_msgs::String msg)
{

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

void ComponentMain::publishTeleoperation(robil2_msgs::String &msg)
{
  _roscomm->publishTeleoperation(msg);
}
