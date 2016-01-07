#include "EventSystem.h"
#include "TaskResult.h"
#include "FSM.h"
#include <ros/ros.h>
using namespace std;
using namespace decision_making;
namespace {
	bool fsm_debug=true;
	#define DIGNOR ros::param::param("/decision_making/debug",fsm_debug,false);if(not fsm_debug)
//	struct DebugModeTracker{
//		EventQueue events;
//		boost::thread_group threads;
//		DebugModeTracker(EventQueue& _events):events(&_events){
//			threads.add_thread(new boost::thread(boost::bind(&DebugModeTracker::track_debug_mode,this)));
//			ros::param::param("/decision_making/debug",fsm_debug,false);
//		}
//		void track_debug_mode(){
//			while(ros::ok() and events.isTerminated()==false){
//				Event e = events.waitEvent();
//				bool changed=false;
//				if(e == Event("/start_fsm_debug")){ fsm_debug_mode = true; changed=true; }
//				if(e == Event("/stop_fsm_debug")){ fsm_debug_mode = false; changed=true; }
//				//if(changed){ cout<<"DEBUG MODE: "<<(fsm_debug_mode?"ON":"OFF")<<endl; }
//			}
//		}
//	};
}
