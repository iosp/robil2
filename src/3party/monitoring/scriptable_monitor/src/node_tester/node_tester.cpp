/*
 * node_tester.cpp
 *
 *  Created on: Jun 3, 2014
 *      Author: dan
 */


#include <ros/ros.h>

#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <list>
#include <set>
#include <algorithm>
#include <fstream>

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <std_msgs/String.h>

using namespace std;
using namespace boost;
using namespace boost::posix_time;
using namespace ros;

string node_ws = "/scriptable_monitor/";

Publisher p_add;
Publisher p_remove;
Publisher p_pause;
Publisher p_resume;

string read_file(string filename){
	stringstream s;
	ifstream file(filename.c_str());
	if(file){
		char c;
		while(file.eof()==false){
			file.read(&c, 1);
			if(c=='\r') continue;
			s<<c;
		}
		return s.str();
		file.close();
	}else{
		cerr<<"ERROR: cann't open file"<<endl;
		return "";
	}
	return "";
}

std_msgs::String to_msg(const stringstream& s){
	std_msgs::String m; m.data = s.str();
	return m;
}
std_msgs::String to_msg(const string& s){
	std_msgs::String m; m.data = s;
	return m;
}
string plp_header;
string PLP_FILE;
string MODULE;

//#define PLP4 "/home/dan/workspace/robil_2/ros_ws/src/robil2/src/monitoring/scriptable_monitor/devel/lib/scriptable_monitor/test4.plp"
//#define PLP4 "/home/dan/workspace/robil_2/ros_ws/src/robil2/src/monitoring/scriptable_monitor/devel/lib/scriptable_monitor/test4rep.plp"
//#define MODULE "Module_1"

//#define PLP4 "/home/dan/workspace/robil_2/ros_ws/src/robil2/src/monitoring/scriptable_monitor/devel/lib/scriptable_monitor/test3.plp"
//#define MODULE "module1"



#define START(F) sleep(1+rand()%3);if(!ros::ok())exit(0);cout<<"add module from file"<<endl;p_add.publish(to_msg(plp_header + read_file(F)));
#define PAUSE sleep(1+rand()%3);if(!ros::ok())exit(0);cout<<"pause module"<<endl;p_pause.publish(to_msg(MODULE));
#define RESUME sleep(1+rand()%3);if(!ros::ok())exit(0);cout<<"resume module"<<endl;p_resume.publish(to_msg(MODULE));
#define STOP sleep(1+rand()%3);if(!ros::ok())exit(0);cout<<"remove module"<<endl;p_remove.publish(to_msg(MODULE));
#define D(X) cout<<"---- "<<#X<<" ----"<<endl; X

void _start_pause_resume_stop(){
	D( START(PLP_FILE) PAUSE RESUME STOP )
}
void _start_resume_pause_stop(){
	D( START(PLP_FILE) RESUME PAUSE STOP )
}
void _start_pause_pause_stop(){
	D( START(PLP_FILE) PAUSE PAUSE STOP )
}
void _start_resume_resume_stop(){
	D( START(PLP_FILE) RESUME RESUME STOP )
}
void _start_resume_pause_resume_pause_stop(){
	D( START(PLP_FILE) RESUME PAUSE RESUME PAUSE STOP )
}
void _start_pause_resume_pause_resume_stop(){
	D( START(PLP_FILE) PAUSE RESUME PAUSE RESUME STOP )
}

int test_bug1(int _a, char** _aa){
	srand(time(NULL));
	PLP_FILE="/home/dan/workspace/robil_2/ros_ws/src/robil2/src/monitoring/scriptable_monitor/devel/lib/scriptable_monitor/test4rep.plp";
	MODULE="Module_1";
	_start_pause_resume_stop();
	_start_resume_pause_stop();
	_start_pause_pause_stop();
	_start_resume_resume_stop();
	_start_resume_pause_resume_pause_stop();
	_start_pause_resume_pause_resume_stop();
	return 0;
}

int test(int _a, char** _aa){
	PLP_FILE="/home/dan/workspace/robil_2/ros_ws/src/robil2/src/monitoring/scriptable_monitor/devel/lib/scriptable_monitor/test4rep.plp";
	MODULE="Module_1";
	_start_pause_resume_stop();
	_start_resume_pause_stop();
	_start_pause_pause_stop();
	_start_resume_resume_stop();
	_start_resume_pause_resume_pause_stop();
	_start_pause_resume_pause_resume_stop();
	return 0;
}


int main(int _a, char** _aa){
	ros::init(_a, _aa, "node_tester");
	ros::NodeHandle node;
	p_add = node.advertise<std_msgs::String>(node_ws+"add_script",10);
	p_remove = node.advertise<std_msgs::String>(node_ws+"delete_script",10);
	p_pause = node.advertise<std_msgs::String>(node_ws+"pause_module",10);
	p_resume = node.advertise<std_msgs::String>(node_ws+"resume_module",10);

	plp_header = "#! type plp\n"
				 "#! name plp_test4\n"
				 "#! interval 1\n";

	if(_a>1 and string(_aa[1])=="bug1"){
		return test_bug1(_a,_aa);
	}

	return test(_a,_aa);
}




