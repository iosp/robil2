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

void ComponentMain::handleMap(std_msgs::String msg)
{
  std::cout<< "PP say:" << msg.data << std::endl;
}
void ComponentMain::handleMissionGlobalPath(std_msgs::String msg)
{
  std::cout<< "PP say:" << msg.data << std::endl;
}
void ComponentMain::handleIEDPosAtt(std_msgs::String msg)
{
  std::cout<< "PP say:" << msg.data << std::endl;
}
void ComponentMain::handlePosAttVel(std_msgs::String msg)
{
  std::cout<< "PP say:" << msg.data << std::endl;
}
void ComponentMain::handleRPPPath(std_msgs::String msg)
{
  std::cout<< "PP say:" << msg.data << std::endl;
}

void ComponentMain::publishLocalPathPlan(std_msgs::String &msg)
{
  _roscomm->publishLocalPathPlan(msg);
}
