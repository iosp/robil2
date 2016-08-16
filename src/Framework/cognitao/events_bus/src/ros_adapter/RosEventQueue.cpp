/*
 * RosEventQueue.cpp
 *
 *  Created on: Nov 6, 2014
 *      Author: dan
 */


#include <cognitao/bus/RosEventQueue.h>

namespace cognitao{
namespace bus{

namespace {
	inline
	ptime toBoost(const ros::Time& rtime){  return rtime.toBoost();
//		 boost::gregorian::date d(1970, boost::gregorian::Jan, 1);
//		 typedef boost::date_time::subsecond_duration<time_duration,1000000000> nanoseconds;
//		 ptime res(d, seconds(rtime.sec)+nanoseconds(rtime.nsec));
//		 return res;
	}
}

RosEventQueue::RosEventQueue(ros::NodeHandle& node, EventQueue* parent,size_t events_max_size, std::string topic_name):
	EventQueue(),node(&node), events_topic_name(topic_name)
{
	connectToRos();
}
RosEventQueue::RosEventQueue(ros::NodeHandle& node, EventQueue& parent_ref,size_t events_max_size, std::string topic_name):
	EventQueue(),node(&node), events_topic_name(topic_name)
{
	connectToRos();
}
RosEventQueue::RosEventQueue(EventQueue* parent,size_t events_max_size, std::string topic_name):
	EventQueue(),node(0), events_topic_name(topic_name)
{
	connectToRos();
}
RosEventQueue::RosEventQueue(EventQueue& parent_ref,size_t events_max_size, std::string topic_name):
	EventQueue(),node(0), events_topic_name(topic_name)
{
	connectToRos();
}

void RosEventQueue::connectToRos(){
	ROS_INFO("Connect to %s: ...", events_topic_name.c_str());
	ros::NodeHandle* _node = node;
	if(not node) _node = new ros::NodeHandle();
	output_events = _node->advertise<events_bus::Msg_Event>(events_topic_name,1000);
	input_events  = _node->subscribe(events_topic_name, 1000 , &RosEventQueue::on_ros_event, this);
	if(not node) delete _node;
	this_thread::sleep(seconds(1));
	ROS_INFO("Connect to  %s: Done", events_topic_name.c_str());
}

void fromMessageEvent( const events_bus::Msg_Event& msg, Event& e )
{
	Event re(msg.event);
	re.parameters(msg.parameters);
	if(msg.stamp.sec>0 or msg.stamp.nsec>0){
		re.stamp( toBoost(msg.stamp) );
	}else{
		re.stamp_to_now();
	}
	re.source(msg.source);
	e = re;
}
void toMessageEvent( const Event& e, events_bus::Msg_Event& msg )
{
	Event re(e);
	//re.stamp_to_now();
	re.stamp_to( toBoost(ros::Time::now()) );
	const bool without_parameters = false;
	msg.event = re.full_name( without_parameters );
	msg.source = re.get_source();
	msg.stamp = ros::Time::fromBoost(re.stamp());
	if(msg.source==""){
		ros::NodeHandle node;
		msg.source = ros::this_node::getName();
	}
	msg.parameters = re.parameters();
}

void RosEventQueue::on_ros_event(const events_bus::Msg_Event::ConstPtr& msg){
	Event re;
	fromMessageEvent(*msg, re);
	push(re);
}

void RosEventQueue::rise(const Event& e){
	if(e == idleEvent()){ EventQueue::rise(e); return; }
	if(e.is_internal()){ EventQueue::rise(e); return; }
	if(e.is_template()) return;
	{
		mutex::scoped_lock l(m);
		if(closed) return;
	}
	if(parent==0){
		events_bus::Msg_Event::Ptr msg(new events_bus::Msg_Event());
		toMessageEvent(e, *msg);
		output_events.publish(msg);
		return;
	}
	parent->rise(e);
}


}
}


