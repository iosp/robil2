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

void ComponentMain::handlePosAttVel( std_msgs::String msg)
{
  std::cout<< "OCU say:" << msg.data << std::endl;
}
void ComponentMain::handleTeleoperation( std_msgs::String msg)
{
  std::cout<< "OCU say:" << msg.data << std::endl;
}
void ComponentMain::handleStatusData( std_msgs::String msg)
{
  std::cout<< "OCU say:" << msg.data << std::endl;
}
void ComponentMain::handleMissionStatus( std_msgs::String msg)
{
  std::cout<< "OCU say:" << msg.data << std::endl;
}
void ComponentMain::handleMap( std_msgs::String msg)
{
  std::cout<< "OCU say:" << msg.data << std::endl;
}
void ComponentMain::handleLocalPathPlan( std_msgs::String msg)
{
  std::cout<< "OCU say:" << msg.data << std::endl;
}
void ComponentMain::handleIEDDetectionEvent( std_msgs::String msg)
{
  std::cout<< "OCU say:" << msg.data << std::endl;
}
void ComponentMain::handleIEDLocation( std_msgs::String msg)
{
  std::cout<< "OCU say:" << msg.data << std::endl;
}

void ComponentMain::publishPositionUpdate(std_msgs::String &msg)
{
  _roscomm->publishPositionUpdate(msg);
}
void ComponentMain::publishMissionPlan(std_msgs::String &msg)
{
  _roscomm->publishMissionPlan(msg);
}
void ComponentMain::publishTeleoperation(std_msgs::String &msg)
{
  _roscomm->publishTeleoperation(msg);
}
void ComponentMain::publishIEDDetectionEvent(std_msgs::String &msg)
{
  _roscomm->publishIEDDetectionEvent(msg);
}
void ComponentMain::publishIEDLocation(std_msgs::String &msg)
{
  _roscomm->publishIEDLocation(msg);
}
