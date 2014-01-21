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

void ComponentMain::handleIEDDetectionEvent(std_msgs::String msg)
{
  std::cout<< "SMME say:" << msg.data << std::endl;
}
void ComponentMain::handleIEDLocation( std_msgs::String msg)
{
  std::cout<< "SMME say:" << msg.data << std::endl;
}
void ComponentMain::handleMissionPlan( std_msgs::String msg)
{
  std::cout<< "SMME say:" << msg.data << std::endl;
}

void ComponentMain::handleStatusData( std_msgs::String msg)
{
  std::cout<< "SMME say:" << msg.data << std::endl;
}

void ComponentMain::publishMissionStatus(std_msgs::String &msg)
{
  _roscomm->publishMissionStatus(msg);
}

void ComponentMain::publishMissionGlobalPath(std_msgs::String &msg)
{
  _roscomm->publishMissionGlobalPath(msg);
}

void ComponentMain::publishIEDPosAtt(std_msgs::String &msg)
{
  _roscomm->publishIEDPosAtt(msg);
}

void ComponentMain::publishExecuteWorkSequenceCommand(std_msgs::String &msg)
{
  _roscomm->publishExecuteWorkSequenceCommand(msg);
}
