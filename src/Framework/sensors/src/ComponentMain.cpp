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


void ComponentMain::publishSensor_SICK(std_msgs::String &msg)
{
  _roscomm->publishSensor_SICK(msg);
}
void ComponentMain::publishSensor_IBEO(std_msgs::String &msg)
{
  _roscomm->publishSensor_IBEO(msg);
}
void ComponentMain::publishSensor_CAM_R(std_msgs::String &msg)
{
  _roscomm->publishSensor_CAM_R(msg);
}
void ComponentMain::publishSensor_CAM_L(std_msgs::String &msg)
{
  _roscomm->publishSensor_CAM_L(msg);
}
void ComponentMain::publishSensor_WIRE(std_msgs::String &msg)
{
  _roscomm->publishSensor_WIRE(msg);
}
void ComponentMain::publishSensor_INSGPS(std_msgs::String &msg)
{
  _roscomm->publishSensor_INSGPS(msg);
}
