/*
 * PathRecorder.cpp
 *
 *  Created on: Nov 19, 2014
 *      Author: dan
 */

#include "PathRecorder.h"

#include "ParameterHandler.h"


PathRecorder::PathRecorder() {
	s_location = ros::Subscriber(node.subscribe("/LOC/Pose", 10, &PathRecorder::on_new_location,this));
	p_plan=ros::Publisher(node.advertise<robil_msgs::Path>("/SMME/GlobalPath",10));
	record = false;
}

PathRecorder::~PathRecorder() {
}

void PathRecorder::on_new_location(const config::SMME::sub::Location& msg){
	if(record) path.push_back(msg.pose.pose);
}


void PathRecorder::start_record(){
	boost::mutex::scoped_lock l(m);
	record = true;
}
void PathRecorder::stop_record(){
	boost::mutex::scoped_lock l(m);
	record = false;
}
void PathRecorder::publish_plan(){
	boost::mutex::scoped_lock l(m);
	config::SMME::pub::GlobalPath path_msg;
	//for(long i=(long)(path.size()-1);i>=0;i--){
	typedef std::list<geometry_msgs::Pose>::reverse_iterator itr;
	for(itr i = itr(path.end()); i!=itr(path.begin());i++){
		geometry_msgs::PoseStamped pose; pose.pose = *i;
		path_msg.waypoints.poses.push_back( pose );
	}
	p_plan.publish(path_msg);
}

void PathRecorder::clean_path(){
	boost::mutex::scoped_lock l(m);
	path.clear();
}

void PathRecorder::communication_ok(){
	path.clear();
}













