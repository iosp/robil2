/*
 * TwistRetranslator.h
 *
 *  Created on: Mar 12, 2014
 *      Author: dan
 */

#ifndef TWISTRETRANSLATOR_H_
#define TWISTRETRANSLATOR_H_

#include <ParameterTypes.h>
#include "ComponentMain.h"

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
