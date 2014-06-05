/*
 * node_tester.cpp
 *
 *  Created on: Jun 3, 2014
 *      Author: dan
 */


#include <ros/ros.h>

#include <plpcpp/PlpCpp.h>
#include <plpcpp/PlpMonitorServer.h>
//#define PLP4 "/home/dan/workspace/robil_2/ros_ws/src/robil2/src/monitoring/scriptable_monitor/devel/lib/scriptable_monitor/test4.plp"
//#define PLP4 "/home/dan/workspace/robil_2/ros_ws/src/robil2/src/monitoring/scriptable_monitor/devel/lib/scriptable_monitor/test4rep.plp"
#define PLP4 "/home/dan/workspace/robil_2/ros_ws/src/robil2/src/monitoring/scriptable_monitor/devel/lib/scriptable_monitor/test3.plp"




int main(int _a, char** _aa){
	ros::init(_a, _aa, "plpcpp_tester");
	ros::NodeHandle node;
	PlpMonitorServer mon(node, true);
	Plp::subscribe(boost::bind(&PlpMonitorServer::on_event,&mon,_1,_2));

	cout<<"Start"<<endl;
	{
		Plp plp(PLP4);
	}
	cout<<"End"<<endl;
	return 0;
}




