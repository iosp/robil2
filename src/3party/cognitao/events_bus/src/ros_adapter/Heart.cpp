/*
 * Heart.cpp
 *
 *  Created on: Nov 9, 2014
 *      Author: dan
 */

#include <cognitao/bus/Heart.h>
#include <std_msgs/String.h>
#include <cognitao/bus/ros_events_bus.h>

namespace cognitao{
namespace monitor{
using namespace bus;

namespace{
	bool random_init(){
		time_duration t = microsec_clock::local_time() - ptime(date(1970,1,1));
		srand( t.total_milliseconds() );
	}
	string uniqNum(){
		static bool b = random_init();
		stringstream s;
		for(int i=0;i<15;i++){
			if(i>0 and i%5==0) s<<'-';
			char c = rand()%16;
			if(c>9) c = (c-10)+'a'; else c+='0';
			s << c;
		}
		return s.str();
	}
}

Heart::Heart(ros::NodeHandle& node, bool use_uniq_id):node(node),use_uniq_id(use_uniq_id) {
	heartbeat = node.advertise<std_msgs::String>("/heartbeat", 10);
	freq = 10;
	node.param("/heartbeat/frequency",freq,freq);

	if(use_uniq_id){
		uniq_name = ros::this_node::getName()+"/"+uniqNum();
	}else{
		uniq_name = ros::this_node::getName();
	}

	th.add_thread(new boost::thread(&Heart::beating,this));
}

Heart::~Heart() {
	th.interrupt_all();
	th.join_all();
}

void Heart::beating(){
	std_msgs::String msg;
	msg.data = uniq_name;

	while(boost::this_thread::interruption_requested()==false){
		boost::system_time stime = boost::get_system_time() + boost::posix_time::microseconds((1.0/freq)*1000000.0);
			std_msgs::String::Ptr pmsg(new std_msgs::String());
			*pmsg = msg;
			heartbeat.publish(pmsg);
		try{
			boost::this_thread::sleep(stime);
		}catch(boost::thread_interrupted& ith){ break; }
	}
}

}
}

