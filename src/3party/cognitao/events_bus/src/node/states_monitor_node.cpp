#include <cognitao/bus/RosEventQueue.h>
#include <ros/ros.h>

#include "../../include/cognitao/bus/RosStatesMonitor.h"

using namespace cognitao;
using namespace cognitao::bus;

void* HBM(ros::NodeHandle& node, EventQueue& events);
void DEL_HBM(void* p);

int main(int a, char** aa){
	ros::init(a, aa, "states_monitor_node");
	ros::NodeHandle node("~");
	RosEventQueue events(node,0);
	node::RosStatesMonitor monitor(node, events);
	ros::AsyncSpinner spinner(0);
	spinner.start();
	void* hbm = HBM(node, events); // create and run heartbeat monitor
	while(ros::ok()){
		monitor.process();
		this_thread::sleep(millisec(100));
	}
	DEL_HBM(hbm); // stop and delete heartbeat monitor
	spinner.stop();
	return 0;
}

