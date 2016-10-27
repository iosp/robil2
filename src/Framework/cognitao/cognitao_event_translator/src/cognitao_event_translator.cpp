#include <ros/ros.h>
#include <std_msgs/String.h>
#include <events_bus/Msg_Event.h>

ros::Publisher pub;

void onStringEvent(const std_msgs::StringPtr & msg)
{
	std::string msg_data = msg->data;

	events_bus::Msg_Event e;
	e.event = "/smme" + msg_data;
	e.stamp = ros::Time::now();
	e.source = "/cognitao_event_translator";

	pub.publish(e);
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "cognitao_event_translator");
	ros::NodeHandle nh("~");

	ros::Subscriber sub = nh.subscribe("/decision_making/events", 10, onStringEvent);
	pub = nh.advertise<events_bus::Msg_Event>("/robil/event_bus/events",10);

	ros::Rate r(30);

	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
