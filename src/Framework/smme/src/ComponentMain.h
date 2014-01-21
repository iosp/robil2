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

        void handleIEDDetectionEvent(std_msgs::String msg);
        void handleIEDLocation( std_msgs::String msg);
        void handleMissionPlan( std_msgs::String msg);
        void handleStatusData( std_msgs::String msg);

        void publishMissionStatus(std_msgs::String &msg);
        void publishMissionGlobalPath(std_msgs::String &msg);
        void publishIEDPosAtt(std_msgs::String &msg);
        void publishExecuteWorkSequenceCommand(std_msgs::String &msg);


};

#endif /* COMPONENTMAIN_H_ */
