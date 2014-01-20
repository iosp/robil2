/*
 * ComponentMain.h
 *
 *  Created on: Jan 16, 2014
 *      Author: yuval
 */

#ifndef COMPONENTMAIN_H_
#define COMPONENTMAIN_H_

#include <robil2_msgs/String.h>

class RosComm;

class ComponentMain {
	RosComm *_roscomm;
public:
	ComponentMain(int argc,char** argv);
	virtual ~ComponentMain();

        void publishSensor_SICK(robil2_msgs::String &msg);
        void publishSensor_IBEO(robil2_msgs::String &msg);
        void publishSensor_CAM_R(robil2_msgs::String &msg);
        void publishSensor_CAM_L(robil2_msgs::String &msg);
        void publishSensor_WIRE(robil2_msgs::String &msg);
        void publishSensor_INSGPS(robil2_msgs::String &msg);
};

#endif /* COMPONENTMAIN_H_ */
