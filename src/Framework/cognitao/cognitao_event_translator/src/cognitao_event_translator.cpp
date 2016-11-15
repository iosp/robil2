#include <ros/ros.h>
#include <std_msgs/String.h>
#include <events_bus/Msg_Event.h>

ros::Publisher pub;

bool isGlobalEvent(const std::string event)
{
	return event.find("ClearMissionBuffer") != std::string::npos;
}

void onStringEvent(const std_msgs::StringPtr & msg)
{
	std::string msg_data = msg->data;

	events_bus::Msg_Event e;
	e.stamp = ros::Time::now();
	e.source = "/cognitao_event_translator";

	if(not isGlobalEvent(msg_data))
	{
		e.event = "/smme" + msg_data;
	}

	pub.publish(e);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cognitao_event_translator");
	ros::NodeHandle nh("~");

	ros::Subscriber sub = nh.subscribe("/decision_making/events", 10, onStringEvent);
	pub = nh.advertise<events_bus::Msg_Event>("/events",10);

	ros::Rate r(30);

	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
