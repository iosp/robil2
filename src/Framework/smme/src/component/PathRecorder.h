/*
 * PathRecorder.h
 *
 *  Created on: Nov 19, 2014
 *      Author: dan
 */

#ifndef PATHRECORDER_H_
#define PATHRECORDER_H_

#include <ros/ros.h>
#include <ParameterTypes.h>
#include <list>


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <robil_msgs/Path.h>

#ifndef COMPONENT
#define COMPONENT context.parameters<Params>().comp
#endif


class PathRecorder {
public:
	PathRecorder();
	virtual ~PathRecorder();


	ros::NodeHandle node;
	ros::Subscriber s_location;
	ros::Publisher  p_plan;

	void on_new_location(const config::SMME::sub::Location& msg);

	boost::mutex m;
	bool record;
	std::list<geometry_msgs::Pose> path;

	void start_record();
	void stop_record();
	void publish_plan();
	void clean_path();

	void communication_ok();

};

#endif /* PATHRECORDER_H_ */
