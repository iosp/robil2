/*
 * ComponentMain.h
 *
 *  Created on: Jan 16, 2014
 *      Author: yuval
 */

#ifndef COMPONENTMAIN_H_
#define COMPONENTMAIN_H_

#include <std_msgs/String.h>

class RosComm;

class ComponentMain {
	RosComm *_roscomm;
public:
	ComponentMain(int argc,char** argv);
	virtual ~ComponentMain();

        void publishSensor_SICK(std_msgs::String &msg);
        void publishSensor_IBEO(std_msgs::String &msg);
        void publishSensor_CAM_R(std_msgs::String &msg);
        void publishSensor_CAM_L(std_msgs::String &msg);
        void publishSensor_WIRE(std_msgs::String &msg);
        void publishSensor_INSGPS(std_msgs::String &msg);
};

#endif /* COMPONENTMAIN_H_ */
