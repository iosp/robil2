/*
 * ComponentMain.cpp
 *
 *  Created on: Jan 16, 2014
 *      Author: yuval
 */

#include "ComponentMain.h"
#include "RosComm.h"

ComponentMain::ComponentMain(int argc,char** argv)
{
	_roscomm=new RosComm(this,argc, argv);
}

ComponentMain::~ComponentMain()
{
	// TODO Auto-generated destructor stub
}

void ComponentMain::handleIEDDetectionEvent(robil2_msgs::String msg)
{

}

void ComponentMain::handleIEDLocation(robil2_msgs::String msg)
{

}

void ComponentMain::publishIEDDetectionEvent(robil2_msgs::String &msg)
{
  _roscomm->publishIEDDetectionEvent(msg);
}

void ComponentMain::publishIEDLocation(robil2_msgs::String &msg)
{
  _roscomm->publishIEDLocation(msg);
}
