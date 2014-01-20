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

        void handleExecuteWorkSequenceCommand( std_msgs::String msg);
        void handlePosAttVel( std_msgs::String msg);
        void handleWiresLengths( std_msgs::String msg);

        void publishBladePosition(std_msgs::String &msg);
        void publishTrottleEffort(std_msgs::String &msg);
        void publishSteeringEffort(std_msgs::String &msg);
        void publishJointsEffort(std_msgs::String &msg);


};

#endif /* COMPONENTMAIN_H_ */
