/*
 * TwistRetranslator.h
 *
 *  Created on: Mar 12, 2014
 *      Author: dan
 */

#ifndef TWISTRETRANSLATOR_H_
#define TWISTRETRANSLATOR_H_

#include "ComponentMain.h"


#ifndef COMPONENT
#define COMPONENT context.parameters<Params>().comp
#endif

#ifndef HEARTBEAT_FREQUANCY
#define HEARTBEAT_FREQUANCY 2 //Hz
#endif

#ifndef HEARTBEAT_FREQUENCY
#define HEARTBEAT_FREQUENCY 2 //Hz
#endif


class TwistRetranslator {
public:
	TwistRetranslator(ComponentMain* comp);
	virtual ~TwistRetranslator();

	void on_twist(const geometry_msgs::Twist::ConstPtr msg);

	ros::NodeHandle node;
	ComponentMain* component;
	ros::Subscriber sub_on_twist_from_base_move;

};

#endif /* TWISTRETRANSLATOR_H_ */
