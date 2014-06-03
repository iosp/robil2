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

int main(int _a, char** _aa){
	ros::init(_a, _aa, "node_tester");
	ros::NodeHandle node;
	p_add = node.advertise<std_msgs::String>(node_ws+"add_script",10);
	p_remove = node.advertise<std_msgs::String>(node_ws+"delete_script",10);
	p_pause = node.advertise<std_msgs::String>(node_ws+"pause_module",10);
	p_resume = node.advertise<std_msgs::String>(node_ws+"resume_module",10);

	string plp_header = "#! type plp\n"
						"#! name plp_test4\n"
						"#! interval 3\n";


	sleep(5);
	cout<<"add module from file"<<endl;
//	p_add.publish(to_msg(read_file("/tmp/t.plp")));
	p_add.publish(to_msg(plp_header + read_file("/home/dan/workspace/robil_2/ros_ws/src/robil2/src/monitoring/scriptable_monitor/devel/lib/scriptable_monitor/test4.plp")));
	sleep(5);
	cout<<"pause module"<<endl;
	p_pause.publish(to_msg("Module_1"));
	sleep(5);
	cout<<"resume module"<<endl;
	p_resume.publish(to_msg("Module_1"));
	sleep(5);
	cout<<"remove module"<<endl;
	p_remove.publish(to_msg("Module_1"));

	return 0;
}




