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


void ComponentMain::handleExecuteWorkSequenceCommand( std_msgs::String msg)
{
  std::cout<< "WSM say:" << msg.data << std::endl;
}

void ComponentMain::handlePosAttVel( std_msgs::String msg)
{
  std::cout<< "WSM say:" << msg.data << std::endl;
}

void ComponentMain::handleWiresLengths( std_msgs::String msg)
{
  std::cout<< "WSM say:" << msg.data << std::endl;
}

void ComponentMain::publishBladePosition(std_msgs::String &msg)
{
  _roscomm->publishBladePosition(msg);
}

void ComponentMain::publishTrottleEffort(std_msgs::String &msg)
{
  _roscomm->publishTrottleEffort(msg);
}
void ComponentMain::publishSteeringEffort(std_msgs::String &msg)
{
  _roscomm->publishSteeringEffort(msg);
}

void ComponentMain::publishJointsEffort(std_msgs::String &msg)
{
  _roscomm->publishJointsEffort(msg);
}
