/*
 * Geometry.h
 *
 *  Created on: Mar 6, 2014
 *      Author: dan
 */

#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include <bullet/LinearMath/btVector3.h>
#include <bullet/LinearMath/btQuaternion.h>
#include <angles/angles.h>
#include "ParameterTypes.h"

inline btVector3 toVector(const geometry_msgs::Pose& A){
	return btVector3(A.position.x, A.position.y, A.position.z);
}
inline btQuaternion toAngles(const geometry_msgs::Pose& A){
	return btQuaternion(A.orientation.x, A.orientation.y, A.orientation.z, A.orientation.w);
}





inline const geometry_msgs::PoseStamped getPoseStamped(const geometry_msgs::PoseWithCovarianceStamped& pos){
	geometry_msgs::PoseStamped p; p.header = pos.header; p.pose = pos.pose.pose;
	return p;
}
inline const geometry_msgs::PoseStamped& getPoseStamped(const geometry_msgs::PoseStamped& pos){
	return pos;
}
inline geometry_msgs::PoseStamped& getPoseStamped(geometry_msgs::PoseStamped& pos){
	return pos;
}

inline const geometry_msgs::Pose& getPose(const geometry_msgs::PoseWithCovarianceStamped& pos){
	return pos.pose.pose;
}
inline geometry_msgs::Pose& getPose(geometry_msgs::PoseWithCovarianceStamped& pos){
	return pos.pose.pose;
}
inline const geometry_msgs::Pose& getPose(const geometry_msgs::PoseStamped& pos){
	return pos.pose;
}
inline geometry_msgs::Pose& getPose(geometry_msgs::PoseStamped& pos){
	return pos.pose;
}
inline const geometry_msgs::Pose& getPose(const geometry_msgs::Pose& pos){
	return pos;
}
inline geometry_msgs::Pose& getPose(geometry_msgs::Pose& pos){
	return pos;
}

inline const geometry_msgs::PoseWithCovariance& getPoseWithCovariance(const geometry_msgs::PoseWithCovarianceStamped& pos){
	return pos.pose;
}
inline geometry_msgs::PoseWithCovariance& getPoseWithCovariance(geometry_msgs::PoseWithCovarianceStamped& pos){
	return pos.pose;
}
inline const geometry_msgs::PoseWithCovariance& getPoseWithCovariance(const geometry_msgs::PoseWithCovariance& pos){
	return pos;
}
inline geometry_msgs::PoseWithCovariance& getPoseWithCovariance(geometry_msgs::PoseWithCovariance& pos){
	return pos;
}

#endif /* GEOMETRY_H_ */
