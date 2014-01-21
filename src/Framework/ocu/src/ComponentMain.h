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


        void handlePosAttVel( std_msgs::String msg);
        void handleTeleoperation( std_msgs::String msg);
        void handleStatusData( std_msgs::String msg);
        void handleMissionStatus( std_msgs::String msg);
        void handleMap( std_msgs::String msg);
        void handleLocalPathPlan( std_msgs::String msg);
        void handleIEDDetectionEvent( std_msgs::String msg);
        void handleIEDLocation( std_msgs::String msg);

        void publishPositionUpdate(std_msgs::String &msg);
        void publishMissionPlan(std_msgs::String &msg);
        void publishTeleoperation(std_msgs::String &msg);
        void publishIEDDetectionEvent(std_msgs::String &msg);
        void publishIEDLocation(std_msgs::String &msg);

};

#endif /* COMPONENTMAIN_H_ */
