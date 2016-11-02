
#include <ros/ros.h>
#include <plpcpp/plp.h>

#define PLPFILE "${ROS:plp_test}/s1.plp"
//#define PLPFILE "/home/dan/workspace/robil_2/ros_ws/src/robil2/src/monitoring/scriptable_monitor/devel/lib/scriptable_monitor/s1.plp"

using namespace plp;



int main(int _a, char** _aa){
	ros::init(_a, _aa, "plptest");
	ros::NodeHandle node;
	plp::init(_a, _aa, node);

	cout<<"Start Program"<<endl;
	sleep(2);

	cout<<"	Start plp module"<<endl;
	Module* plp_module_ptr = new Module(PLPFILE);
	sleep(2);
	{
		cout<<"		Start goal module"<<endl;
		Module::Iteration goal = plp_module_ptr->goal_achievement();
		sleep(5);
		cout<<"		Stop goal"<<endl;
	}
	sleep(2);
	cout<<"	Stop plp module"<<endl;
	delete plp_module_ptr;

	sleep(2);
	cout<<"End Program"<<endl;
	return 0;
}




