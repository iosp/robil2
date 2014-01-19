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

void ComponentMain::handleBladePosition(robil2_msgs::String msg)
{

}

void ComponentMain::handlePosAttVel(robil2_msgs::String msg)
{

}

void ComponentMain::handleLaser(robil2_msgs::String msg)
{

}

void ComponentMain::handleCamera(robil2_msgs::String msg)
{

}

void ComponentMain::publishMap(robil2_msgs::String &msg)
{
  _roscomm->publishMap(msg);
}

void ComponentMain::publishMiniMap(robil2_msgs::String &msg)
{
  _roscomm->publishMiniMap(msg);
}
