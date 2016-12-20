/*
 * Types.h
 *
 *  Created on: Mar 5, 2014
 *      Author: dan
 */

#ifndef TYPES_H_
#define TYPES_H_

#include <ParameterTypes.h>

config::SMME::pub::GlobalPath
	extract_path(
			const config::SMME::sub::AssignNavTask& task
	)
{
	/** NOTE:
	 * Twist limitation and time stamps are not used yet.
	 */
	config::SMME::pub::GlobalPath path;
	path.id = task.task_id;
	path.waypoints.poses.resize(task.waypoints.size());
	path.waypoints.header = task.header;
	path.is_heading_defined=true;
	path.heading = task.heading_at_last_point;
	for(size_t i=0;i<task.waypoints.size();i++){
		nav_msgs::Odometry wp = task.waypoints[i];
		path.waypoints.poses[i].pose=wp.pose.pose;
	}
	return path;
}

#endif /* TYPES_H_ */
