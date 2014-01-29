/*
 * ComponentMain.cpp
 *
 *  Created on: Jan 16, 2014
 *      Author: yuval
 */

#include "ComponentMain.h"
#include "../roscomm/RosComm.h"

ComponentMain::ComponentMain(int argc,char** argv)
{
	_roscomm = new RosComm(this,argc, argv);
}

ComponentMain::~ComponentMain() {
	if(_roscomm) delete _roscomm;
}

void ComponentMain::handleMap(const config::PP::sub::Map& msg)
{
  std::cout<< "PP say:" << msg.data << std::endl;
}
void ComponentMain::handleMissionGlobalPath(const config::PP::sub::MissionGlobalPath& msg)
{
  std::cout<< "PP say:" << msg.data << std::endl;
}
void ComponentMain::handleIEDPosAtt(const config::PP::sub::IEDPosAtt& msg)
{
  std::cout<< "PP say:" << msg.data << std::endl;
}
void ComponentMain::handlePosAttVel(const config::PP::sub::PosAttVel& msg)
{
  std::cout<< "PP say:" << msg.data << std::endl;
}
void ComponentMain::handleRPPPath(const config::PP::sub::RPPPath& msg)
{
  std::cout<< "PP say:" << msg.data << std::endl;
}

void ComponentMain::publishLocalPathPlan(config::PP::pub::LocalPathPlan& msg)
{
  _roscomm->publishLocalPathPlan(msg);
}
