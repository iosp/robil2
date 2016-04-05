/*
 * Heart.cpp
 *
 *  Created on: Nov 9, 2014
 *      Author: dan
 */

#include <std_msgs/String.h>
#include <cognitao/bus/ros_events_bus.h>
#include <algorithm>

using namespace cognitao;
using namespace cognitao::bus;
using namespace cognitao::monitor;

namespace {
	set<string> diff(const set<string>& s1, const set<string>& s2){
		set<string> res;
		for(set<string>::const_iterator i=s1.begin();i!=s1.end();i++){
			if( s2.find(*i)!=s2.end() ) continue;
			res.insert(*i);
		}
		return res;
	}
}
class HeartbeatMonitor{
public:
	ros::NodeHandle& node;
	EventQueue events;
	ros::Subscriber hearbeat;
	double freq;
	mutex m;
	thread_group threads;
	set<string> all_nodes;
	set<string> updated_nodes;

	HeartbeatMonitor(ros::NodeHandle& node, EventQueue& events):node(node), events(events){
		hearbeat = node.subscribe("/heartbeat",10,&HeartbeatMonitor::on_heartbeat,this);
		threads.add_thread(new thread(&HeartbeatMonitor::checker,this));
		freq = 10;
		node.param("/heartbeat/frequency",freq,freq);
	}
	~HeartbeatMonitor(){
		threads.interrupt_all();
		threads.join_all();
	}

	void checker(){
		try{
			while(ros::ok() and this_thread::interruption_requested()==false){
				system_time stime = get_system_time() + microseconds((1.0/freq)*1000000.0*3);
				{
					//cout<<"[d] check"<<endl;
					mutex::scoped_lock l(m);
					set<string> not_updated = diff(all_nodes, updated_nodes	);
					//cout<<"[d] all.size="<<all_nodes.size()<<", updated.size="<<updated_nodes.size()<<", not_updated.size="<<not_updated.size()<<endl;
					for(set<string>::iterator i=not_updated.begin();i!=not_updated.end();i++){
						events << StatesMonitor::endAll_event(Event::context_t("/nodes"+*i),"timeout");
					}
					all_nodes = updated_nodes;
					updated_nodes.clear();
				}
				try{
					this_thread::sleep(stime);
				}
				catch( const boost::thread_interrupted& tin)
				{
					break;
				}
				catch(const std::exception& gex)
				{
					std::cout<<"[w] exception during checker process (sleep) \n"<<gex.what()<<std::endl;
				}
				catch(...){ break; }
			}
		}
		catch (const boost::thread_interrupted& thread_interrupted_ex) {
			std::cout<<"[d] checker thread interrupted"<<std::endl;
		}
	}
	void on_heartbeat(const std_msgs::String::ConstPtr& msg){
		mutex::scoped_lock l(m);
		if(all_nodes.find(msg->data)==all_nodes.end()){
			events << StatesMonitor::begin_event(Event::context_t("/nodes"+msg->data));
			all_nodes.insert(msg->data);
		}
		//cout<<"[d] updated "<< (msg->data)<< endl;
		updated_nodes.insert(msg->data);
	}
};

void* HBM(ros::NodeHandle& node, EventQueue& events){ return new HeartbeatMonitor(node, events); }
void DEL_HBM(void* p){ delete (HeartbeatMonitor*)p; }

int haertbeat_main(int a, char** aa){
	ros::init(a, aa, "heartbeat_monitor");
	ros::NodeHandle node("~");
	RosEventQueue events(node, 0);
	HeartbeatMonitor monitor(node, events);
	ros::spin();
	return 0;
}
