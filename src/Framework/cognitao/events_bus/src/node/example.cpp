//============================================================================
// Name        : Events.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <ros/ros.h>
#include <cognitao/bus/ros_events_bus.h>

using namespace cognitao::monitor;
using namespace cognitao::bus;


namespace {
inline Event setSource(const Event& e){
	static ros::NodeHandle node;
	Event ee(e);
	ee.set_source(ros::this_node::getName());
	return ee;
}
}

class Mission1:public Model{
public:
	string state;
	string sub_state;
	Mission1(){
	}
	void init(EventRiser& riser){
		state = "init";
		sub_state = "step1";
		riser << setSource(StatesMonitor::begin_event(Event::context_t("/mission/m1/init")));
		riser << setSource(StatesMonitor::begin_event(Event::context_t("/mission/m1/init/step1")));
	}
	void on_event(const Event& e, EventRiser& riser){
		if(e == "*/m1/next_step"){
			if(state=="init" and sub_state=="step1"){
				sub_state="step2";
				riser.rise(StatesMonitor::end_event(Event::context_t("/mission/m1/init/step1"),"success"));
				riser.rise(StatesMonitor::begin_event(Event::context_t("/mission/m1/init/step2")));
			}else
			if(state=="init" and sub_state=="step2"){
				state = "stopped";
				sub_state = "undef";
				riser << setSource(StatesMonitor::end_event(Event::context_t("/mission/m1/init"),"success"));
				riser << setSource(StatesMonitor::end_event(Event::context_t("/mission/m1/init/step2"),"success"));
				riser << setSource(StatesMonitor::begin_event(Event::context_t("/mission/m1/stopped")));
			}
		}
	}
};

void thread_monitoring(StatesMonitor& monitor){
	try{
		while(this_thread::interruption_requested()==false){
			monitor.process();
			this_thread::sleep(millisec(100));
		}
	}catch(...){}
}



int main(int a, char** aa) {
	ros::init(a, aa, "events_bus_example");
	ros::NodeHandle node;
	ros::AsyncSpinner spinner(0);
	spinner.start();

	RosEventQueue events(node,0);
	events.run_idle_timer(millisec(100));
	StatesMonitor monitor(events); // create local states monitor
	cognitao::monitor::StatesMonitorClient ros_monitor(node); // connect to remove states monitor
	Heart heart(node); // create heart of node

//#define E(X)	events << Event(X); cout<<X<<endl; this_thread::sleep(posix_time::millisec(500));
#define E(X)	events << Event(X); cout<<X<<endl; this_thread::sleep(posix_time::seconds(2));


	Mission1 mission1;
	monitor.add(mission1);
	thread mthread(thread_monitoring,ref(monitor));

	E("/mission/m1/next_step");
	E(StatesMonitor::begin_event("/mission/m2/init"));
	E("/mission/m2/begin");
	E("/mission/m1/next_step");
	E(StatesMonitor::end_event("/mission/m2/init", "success"));
	E(StatesMonitor::begin_event("/mission/m2/init"));

	this_thread::sleep(milliseconds(200));

	cout << monitor << endl;
	cout << monitor.is_active("/mission/m2/init") << endl;
	cout << monitor.is_active("*/m1/init") << endl;
	cout << monitor.is_exists("/mission/m2/init") << endl;

	cout << ros_monitor.is_active("/mission/m2/init") << endl;
	cout << ros_monitor.is_active("*/m1/init") << endl;
	cout << ros_monitor.is_exists("/mission/m2/init") << endl;

	mthread.interrupt();
	mthread.join();
	spinner.stop();
	cout<<"End"<<endl;
	return 0;
}
