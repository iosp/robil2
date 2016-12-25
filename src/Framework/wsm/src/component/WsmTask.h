#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <ros/ros.h>


#include <diagnostic_msgs/DiagnosticArray.h>
#include <robil_msgs/AssignManipulatorTask.h>

#ifndef COMPONENT
#define COMPONENT context.parameters<Params>().comp
#endif

#ifndef HEARTBEAT_FREQUANCY
#define HEARTBEAT_FREQUANCY 2 //Hz
#endif

#ifndef HEARTBEAT_FREQUENCY
#define HEARTBEAT_FREQUENCY 2 //Hz
#endif


using namespace std ;
class ComponentMain;
class Vec3D ;
class WsmTask {

	string _status ;
	int _taskid ;
	int _cur_step ;
	string _step_status;
	robil_msgs::AssignManipulatorTask * _cur_WSD;
	ComponentMain* _comp;

public:

	WsmTask(ComponentMain* comp);
	WsmTask(int taskid , int cur_step ,const robil_msgs::AssignManipulatorTask& cur_WSD , ComponentMain* comp);
	WsmTask(const WsmTask& other);
	~WsmTask();
	void pauseTask();
	void Save_state();
	void Load_state();
	void Update_step();
	void Set_task_status(string status);
	void Set_step_id(int index);
	void Set_Task_WSD(const robil_msgs::AssignManipulatorTask &WSD);
	robil_msgs::AssignManipulatorTaskStep* Get_step();
	robil_msgs::AssignManipulatorTask * Get_WSD();
	int Get_Task_id();
	int Get_cur_step_index();
	void publish_step_diag(int before , int exit_status);
	void push_key_value(diagnostic_msgs::DiagnosticStatus container , std::string key , std::string value);
	string Get_status();
	string Get_step_id();
	void execute_next_step();
	void inter_step_sleep(unsigned int duration_at_end);
	int handle_type_1();
	int handle_type_2();
	int handle_type_3();
	int handle_type_4();
	int handle_type_5();
	double find_Map_max(int x);
	Vec3D deriveMapPixel (tf::StampedTransform blade2body);
	void blade_correction();
	void monitor_time(double time);
	void debug();
};


