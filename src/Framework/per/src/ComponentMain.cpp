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


void ComponentMain::handleSensor_SICK(robil2_msgs::String msg)
{

}
void ComponentMain::handleSensor_IBEO(robil2_msgs::String msg)
{

}
void ComponentMain::handleSensor_CAM_R(robil2_msgs::String msg)
{

}
void ComponentMain::handleSensor_CAM_L(robil2_msgs::String msg)
{

}
void ComponentMain::handleSensor_WIRE(robil2_msgs::String msg)
{

}
void ComponentMain::handleSensor_INSGPS(robil2_msgs::String msg)
{

}

void ComponentMain::publishWiresLengths(robil2_msgs::String &msg)
{
  _roscomm->publishWiresLengths(msg);
}
void ComponentMain::publishCamera(robil2_msgs::String &msg)
{
  _roscomm->publishCamera(msg);
}
void ComponentMain::publishLaser(robil2_msgs::String &msg)
{
  _roscomm->publishLaser(msg);
}
void ComponentMain::publishINS(robil2_msgs::String &msg)
{
  _roscomm->publishINS(msg);
}
void ComponentMain::publishGPS(robil2_msgs::String &msg)
{
  _roscomm->publishGPS(msg);
}
void ComponentMain::publishTF(robil2_msgs::String &msg)
{
  _roscomm->publishTF(msg);
}
