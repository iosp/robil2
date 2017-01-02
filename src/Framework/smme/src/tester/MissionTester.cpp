/*
 * MissionTester.cpp
 *
 *  Created on: Mar 13, 2014
 *      Author: dan
 */

#include "MissionTester.h"
#include "Colors.hpp"

std::string operator+(std::string s, const int& t){ stringstream ss; ss<<t; return s+ss.str(); }

MissionTester::MissionTester() {

	p_NavTask = node.advertise<NavTask>("/OCU/SMME/NavigationTask",10);
	p_Mission = node.advertise<Mission>("/OCU/SMME/MissionPlan",10);

}

MissionTester::~MissionTester() {

}

NavTask createTask(int n){
	static int seq=0;
	NavTask task;
	task.task_id=string("task_")+n;
	task.task_description="description of task";
	task.header.frame_id="map";
	task.header.stamp=ros::Time::now();
	task.header.seq = seq++;
	task.heading_at_last_point = 3.14/2;
	typedef NavTask::_waypoints_type::value_type Value;
	Value v;
	v.header = task.header;

	#	define PUSH(N) \
		v.pose.pose.position.x=N;\
		v.pose.pose.position.y=N+0.1;\
		task.waypoints.push_back(v);

		PUSH(1+n)
		PUSH(10+n)
		PUSH(100+n)

	#	undef PUSH
	return task;
}

Mission createEmptyMission(){
	static int seq=0;
	Mission mission;
	mission.mission_id = "empty_mission";
	mission.mission_description = "empty mission for testing rejection mechanism";
	mission.header.frame_id="map";
	mission.header.stamp=ros::Time::now();
	mission.header.seq = seq++;

	return mission;
}

Mission createMission_1(){
	static int seq=0;
	Mission mission;
	mission.mission_id = "mission_1";
	mission.mission_description = "description of mission";
	mission.header.frame_id="map";
	mission.header.stamp=ros::Time::now();
	mission.header.seq = seq++;
	typedef Mission::_tasks_type::value_type Value;
	mission.tasks.push_back(Value());
	mission.tasks.back().task_id = "task_0";
	return mission;
}
Mission createMission_2(){
	static int seq=0;
	Mission mission;
	mission.mission_id = "mission_2";
	mission.mission_description = "description of mission";
	mission.header.frame_id="map";
	mission.header.stamp=ros::Time::now();
	mission.header.seq = seq++;
	typedef Mission::_tasks_type::value_type Value;
	mission.tasks.push_back(Value());
	mission.tasks.back().task_id = "task_0";
	mission.tasks.push_back(Value());
	mission.tasks.back().task_id = "task_1";
	mission.tasks.push_back(Value());
	mission.tasks.back().task_id = "task_3";
	return mission;
}

#define TEST(t1){\
	string _##t1=t1();\
	std::cout<<"Test "#t1" ... ";\
	if(c.call(msg)){\
		if(_##t1!=msg.response.states){\
			std::cout<<std::endl<<BOLDRED<<"TEST FAULT"<<RESET<<std::endl;\
			std::cout<<RED<<"--- TARGET ---:\n"<<_##t1<<"--- ACTUAL ---:\n"<<msg.response.states<<RESET<<std::endl;\
			return false;\
		}else{\
			std::cout<<" pass"<<std::endl;\
		}\
	}else{\
		std::cout<<std::endl<<BOLDRED<<"TEST FAULT"<<RESET<<std::endl;\
		std::cout<<RED<<"ERROR: Service \"/mission_state\" not available."<<RESET<<std::endl;\
		return false;\
	}}

string t1(){
	return
"NTasks:\n\
   task_1 {wp=3}\n\
   task_2 {wp=3}\n\
   task_3 {wp=3}\n\
MTasks:\n\
Missions:\n\
   mission_1 {tsk=1}\n\
   mission_2 {tsk=3}\n\
Missions States:\n\
   mission_1{ms=loaded:ti=0:ts=loaded}\n\
   mission_2{ms=loaded:ti=0:ts=loaded}\n";
}
string t2(){
	return
"NTasks:\n\
   task_1 {wp=3}\n\
   task_2 {wp=3}\n\
   task_3 {wp=3}\n\
MTasks:\n\
Missions:\n\
   mission_1 {tsk=1}\n\
   mission_2 {tsk=3}\n\
Missions States:\n\
   mission_1{ms=pending:ti=0:ts=pending}\n\
   mission_2{ms=pending:ti=0:ts=pending}\n";
}
string t3(){
	return
"NTasks:\n\
   task_1 {wp=3}\n\
   task_2 {wp=3}\n\
   task_3 {wp=3}\n\
MTasks:\n\
Missions:\n\
   mission_1 {tsk=1}\n\
   mission_2 {tsk=3}\n\
Missions States:\n\
   mission_1{ms=spooling:ti=0:ts=spooling}\n\
   mission_2{ms=pending:ti=0:ts=pending}\n";
}
string t4(){
	return
"NTasks:\n\
   task_1 {wp=3}\n\
   task_2 {wp=3}\n\
   task_3 {wp=3}\n\
MTasks:\n\
Missions:\n\
   mission_1 {tsk=1}\n\
   mission_2 {tsk=3}\n\
Missions States:\n\
   mission_1{ms=paused:ti=0:ts=pending}\n\
   mission_2{ms=pending:ti=0:ts=pending}\n";
}
string t5(){
	return
"NTasks:\n\
   task_1 {wp=3}\n\
   task_2 {wp=3}\n\
   task_3 {wp=3}\n\
MTasks:\n\
Missions:\n\
   mission_1 {tsk=1}\n\
   mission_2 {tsk=3}\n\
Missions States:\n\
   mission_1{ms=paused:ti=0:ts=pending}\n\
   mission_2{ms=spooling:ti=0:ts=spooling}\n";
}
string t6(){
	return
"NTasks:\n\
   task_1 {wp=3}\n\
   task_2 {wp=3}\n\
   task_3 {wp=3}\n\
MTasks:\n\
Missions:\n\
   mission_1 {tsk=1}\n\
   mission_2 {tsk=3}\n\
Missions States:\n\
   mission_1{ms=paused:ti=0:ts=pending}\n\
   mission_2{ms=spooling:ti=1:ts=spooling}\n";
}
string t7(){
	return
"NTasks:\n\
   task_1 {wp=3}\n\
   task_2 {wp=3}\n\
   task_3 {wp=3}\n\
MTasks:\n\
Missions:\n\
   mission_1 {tsk=1}\n\
   mission_2 {tsk=3}\n\
Missions States:\n\
   mission_1{ms=spooling:ti=0:ts=spooling}\n\
   mission_2{ms=paused:ti=1:ts=pending}\n";
}
string t8(){
	return
"NTasks:\n\
   task_1 {wp=3}\n\
   task_2 {wp=3}\n\
   task_3 {wp=3}\n\
MTasks:\n\
Missions:\n\
   mission_1 {tsk=1}\n\
   mission_2 {tsk=3}\n\
Missions States:\n\
   mission_1{ms=finished:ti=0:ts=pending}\n\
   mission_2{ms=paused:ti=1:ts=pending}\n";
}

string t9(){
	return
"NTasks:\n\
   task_1 {wp=3}\n\
   task_2 {wp=3}\n\
   task_3 {wp=3}\n\
MTasks:\n\
Missions:\n\
   mission_1 {tsk=1}\n\
   mission_2 {tsk=3}\n\
Missions States:\n\
   mission_1{ms=finished:ti=0:ts=pending}\n\
   mission_2{ms=spooling:ti=1:ts=spooling}\n";
}
string t10(){
	return
"NTasks:\n\
   task_1 {wp=3}\n\
   task_2 {wp=3}\n\
   task_3 {wp=3}\n\
MTasks:\n\
Missions:\n\
   mission_1 {tsk=1}\n\
   mission_2 {tsk=3}\n\
Missions States:\n\
   mission_1{ms=finished:ti=0:ts=pending}\n\
   mission_2{ms=aborted:ti=1:ts=pending}\n";
}



#define SEND(E){\
	std_msgs::String m; m.data = E;\
	p.publish(m);\
}

#define SLEEP(S) this_thread::sleep(seconds(S));

#include <robil_msgs/MissionState.h>
bool MissionTester::test(){
	NavTask task1 = createTask(1);
	p_NavTask.publish(task1);
	NavTask task2 = createTask(2);
	p_NavTask.publish(task2);
	NavTask task3 = createTask(3);
	p_NavTask.publish(task3);
	Mission mission_1 = createMission_1();
	p_Mission.publish(mission_1);
	Mission mission_2 = createMission_2();
	p_Mission.publish(mission_2);

	bool error = false;

	ServiceClient c = node.serviceClient<robil_msgs::MissionState>("/mission_state");
	Publisher p = node.advertise<std_msgs::String>("/decision_making/events",10);
	robil_msgs::MissionState msg;

	SLEEP(1)
	TEST(t1)

	SLEEP(3)
	TEST(t2)

	SEND("/mission/mission_1/StartMission")
	SLEEP(1)
	TEST(t3)

	SEND("/mission/mission_1/PauseMission")
	SLEEP(1)
	TEST(t4)

	SEND("/mission/mission_2/StartMission")
	SLEEP(1)
	TEST(t5)

	SEND("/CompleteTask")
	SLEEP(1)
	TEST(t6)

	SEND("/mission/mission_2/PauseMission")
	SLEEP(1)
	SEND("/mission/mission_1/ResumeMission")
	SLEEP(1)
	TEST(t7)

	SEND("/CompleteTask")
	SLEEP(1)
	TEST(t8)

	SEND("/mission/mission_2/ResumeMission")
	SLEEP(1)
	TEST(t9)

	SEND("/AbortTask")
	SLEEP(1)
	TEST(t10)


	std::cout<<BOLDGREEN<<"TEST PASS"<<RESET<<std::endl;
	return true;
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
