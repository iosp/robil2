/*
 * TwistRetranslator.cpp
 *
 *  Created on: Mar 12, 2014
 *      Author: dan
 */

#include "TwistRetranslator.h"

TwistRetranslator::TwistRetranslator(ComponentMain* comp)
	:component(comp)
{

	sub_on_twist_from_base_move = node.subscribe("/cmd_vel",10,&TwistRetranslator::on_twist,this);

}

TwistRetranslator::~TwistRetranslator() {

}

void TwistRetranslator::on_twist(const geometry_msgs::Twist::ConstPtr msg){
	config::WPD::pub::WPDVelocity out_msg;
	out_msg.header.stamp = ros::Time::now();
	out_msg.header.frame_id = "map";
	out_msg.twist = *msg;
	component->publishWPDVelocity(out_msg);
}



