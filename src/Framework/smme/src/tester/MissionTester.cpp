/*
 * MissionTester.cpp
 *
 *  Created on: Mar 13, 2014
 *      Author: dan
 */

#include "MissionTester.h"

template<class A> std::string operator+(std::string s, A t){ stringstream ss(s); ss<<t; return ss.str(); }

MissionTester::MissionTester() {

	p_NavTask = node.advertise<NavTask>(fetchParam(&node,"SMME","AssignNavTask","sub"),10);
	p_Mission = node.advertise<Mission>(fetchParam(&node,"SMME","AssignMission","sub"),10);

}

MissionTester::~MissionTester() {

}

NavTask createTask(int n){
	static int seq=0;
	NavTask task;
	task.task_id="task_"+n;
	task.task_description="description of task";
	task.header.frame_id="map";
	task.header.stamp=ros::Time::now();
	task.header.seq = seq++;
	task.heading_at_last_point = 3.14/2;
	typedef NavTask::_waypoints_type::value_type Value;
	Value v;
	v.header = task.header;

	#	define PUSH(N) \
		v.pose.pose.position.x=1;\
		v.pose.pose.position.y=1;\
		task.waypoints.push_back(v);

		PUSH(1+n)
		PUSH(10+n)
		PUSH(100+n)

	#	undef PUSH
	return task;
}

Mission createMission(){
	static int seq=0;
	Mission mission;
	mission.mission_id = "mission_1";
	mission.mission_description = "description of mission";
	mission.header.frame_id="map";
	mission.header.stamp=ros::Time::now();
	mission.header.seq = seq++;

	return mission;
}

void MissionTester::test(){
	NavTask task1 = createTask(1);
	p_NavTask.publish(task1);
	NavTask task2 = createTask(2);
	p_NavTask.publish(task2);
	NavTask task3 = createTask(3);
	p_NavTask.publish(task3);
	Mission mission = createMission();
	p_Mission.publish(mission);
}

int main(int a, char** aa){
	ros::init(a,aa, "MissionTester");
	ros::AsyncSpinner sp(2);
	sp.start();

	MissionTester tester;
	ROS_INFO("Pause before test");
	this_thread::sleep(seconds(5));
	ROS_INFO("Start test");

	tester.test();

	this_thread::sleep(seconds(5));
	ROS_INFO("End test");
	return 0;
}
