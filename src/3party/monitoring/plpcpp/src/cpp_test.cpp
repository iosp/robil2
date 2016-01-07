/*
 * node_tester.cpp
 *
 *  Created on: Jun 3, 2014
 *      Author: dan
 */


#include <ros/ros.h>

#include <plpcpp/plp.h>

//#define PLP4 "/home/dan/workspace/robil_2/ros_ws/src/robil2/src/monitoring/scriptable_monitor/devel/lib/scriptable_monitor/test4.plp"
#define PLP4 "${ROS:scriptable_monitor}/test4.plp"
//#define PLP4 "/home/dan/workspace/robil_2/ros_ws/src/robil2/src/monitoring/scriptable_monitor/devel/lib/scriptable_monitor/test4rep.plp"
//#define PLP3 "/home/dan/workspace/robil_2/ros_ws/src/robil2/src/monitoring/scriptable_monitor/devel/lib/scriptable_monitor/test3.plp"
#define PLP3 "${ROS:scriptable_monitor}/test3.plp"


using namespace plp;



int main(int _a, char** _aa){
//	cout<<"> "<< (std::string) RosPack().find_file("${ROS:pp}/*.cpp")[0] <<endl;

	ros::init(_a, _aa, "plpcpp_tester");
	ros::NodeHandle node;
	plp::init(_a, _aa, node);

	cout<<"Start"<<endl;
	sleep(1);
	Module p(PLP4);
	sleep(5);
	cout<<"Start --- "<<endl;
	if(true){
		Module plp(PLP3);
		for(int i=0;i<3;i++){
			sleep(2);
			Module::Iteration itr=plp.goal_achievement();
			cout<<"--"<<endl;
			sleep(1);
		}
		sleep(1);
	}
	cout<<"End"<<endl;
	return 0;
}




