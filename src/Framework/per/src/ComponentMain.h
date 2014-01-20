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

        void handleSensor_SICK(robil2_msgs::String msg);
        void handleSensor_IBEO(robil2_msgs::String msg);
        void handleSensor_CAM_R(robil2_msgs::String msg);
        void handleSensor_CAM_L(robil2_msgs::String msg);
        void handleSensor_WIRE(robil2_msgs::String msg);
        void handleSensor_INSGPS(robil2_msgs::String msg);

        void publishWiresLengths(robil2_msgs::String &msg);
        void publishCamera(robil2_msgs::String &msg);
        void publishLaser(robil2_msgs::String &msg);
        void publishINS(robil2_msgs::String &msg);
        void publishGPS(robil2_msgs::String &msg);
        void publishTF(robil2_msgs::String &msg);
};

#endif /* COMPONENTMAIN_H_ */
