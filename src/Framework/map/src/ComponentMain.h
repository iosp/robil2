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



        void handleBladePosition(std_msgs::String msg);
        void handlePosAttVel(std_msgs::String msg);
        void handleLaser(std_msgs::String msg);
        void handleCamera(std_msgs::String msg);

        void publishMap(std_msgs::String &msg);
        void publishMiniMap(std_msgs::String &msg);
};

#endif /* COMPONENTMAIN_H_ */
