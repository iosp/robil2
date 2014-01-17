/*
 * RosComm.h
 *
 *  Created on: Jan 16, 2014
 *      Author: yuval
 */

#ifndef ROSCOMM_H_
#define ROSCOMM_H_

#include <ros/ros.h>
#include <robil2_msgs/String.h>

class RosComm {

	ros::Publisher * _genericTopicPublisher;
	ros::Subscriber * _genericTopicSubscriber;
public:
	RosComm();
	virtual ~RosComm();
};


#endif /* ROSCOMM_H_ */
