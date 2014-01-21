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


void ComponentMain::handleSensor_SICK(std_msgs::String msg)
{
  std::cout<< "PER say:" << msg.data << std::endl;
}
void ComponentMain::handleSensor_IBEO(std_msgs::String msg)
{
  std::cout<< "PER say:" << msg.data << std::endl;
}
void ComponentMain::handleSensor_CAM_R(std_msgs::String msg)
{
  std::cout<< "PER say:" << msg.data << std::endl;
}
void ComponentMain::handleSensor_CAM_L(std_msgs::String msg)
{
  std::cout<< "PER say:" << msg.data << std::endl;
}
void ComponentMain::handleSensor_WIRE(std_msgs::String msg)
{
  std::cout<< "PER say:" << msg.data << std::endl;
}
void ComponentMain::handleSensor_INSGPS(std_msgs::String msg)
{
  std::cout<< "PER say:" << msg.data << std::endl;
}

void ComponentMain::publishWiresLengths(std_msgs::String &msg)
{
  _roscomm->publishWiresLengths(msg);
}
void ComponentMain::publishCamera(std_msgs::String &msg)
{
  _roscomm->publishCamera(msg);
}
void ComponentMain::publishLaser(std_msgs::String &msg)
{
  _roscomm->publishLaser(msg);
}
void ComponentMain::publishINS(std_msgs::String &msg)
{
  _roscomm->publishINS(msg);
}
void ComponentMain::publishGPS(std_msgs::String &msg)
{
  _roscomm->publishGPS(msg);
}
void ComponentMain::publishTF(std_msgs::String &msg)
{
  _roscomm->publishTF(msg);
}
