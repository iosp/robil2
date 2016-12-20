/*
 * RosEventQueue.h
 *
 *  Created on: Nov 6, 2014
 *      Author: dan
 */

#ifndef ROSEVENTQUEUE_H_
#define ROSEVENTQUEUE_H_

#include <cognitao/bus/EventQueue.h>
#include <ros/ros.h>
#include <events_bus/Msg_Event.h>

namespace cognitao {
namespace bus {

namespace message {
typedef events_bus::Msg_Event Event;
}

class RosEventQueue: public EventQueue {
public:
	typedef boost::shared_ptr<RosEventQueue> Ptr;

	ros::NodeHandle* node;
	ros::Subscriber input_events;
	ros::Publisher output_events;
	std::string events_topic_name;
	RosEventQueue(ros::NodeHandle& node, EventQueue* parent = 0,
			size_t events_max_size = 1000, std::string topic_name = "/events");
	RosEventQueue(ros::NodeHandle& node, EventQueue& parent_ref,
			size_t events_max_size = 1000, std::string topic_name = "/events");
	RosEventQueue(EventQueue* parent = 0, size_t events_max_size = 1000,
			std::string topic_name = "/events");
	RosEventQueue(EventQueue& parent_ref, size_t events_max_size = 1000,
			std::string topic_name = "/events");
	void connectToRos();
	void on_ros_event(const message::Event::ConstPtr& msg);
	void rise(const Event& e);
};

void fromMessageEvent(const message::Event& msg, Event& e);
void toMessageEvent(const Event& e, message::Event& msg);

}
}

#endif /* ROSEVENTQUEUE_H_ */
