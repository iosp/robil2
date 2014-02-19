/*
 * HeartBeatMonitor.h
 *
 *  Created on: Feb 19, 2014
 *      Author: dan
 */

#ifndef HEARTBEATMONITOR_H_
#define HEARTBEATMONITOR_H_

#include <ros/ros.h>
#include <string>
#include <map>
#include <boost/thread.hpp>
#include <diagnostic_msgs/DiagnosticArray.h>

#define LIMIT_HZ 1

class HeartBeatMonitor {
public:
	HeartBeatMonitor();
	virtual ~HeartBeatMonitor();

	void run();

	void on_heartbeat(std::string component);
	diagnostic_msgs::DiagnosticStatus on_timeout(std::string component);
	diagnostic_msgs::DiagnosticStatus on_new_component(std::string component);
	diagnostic_msgs::DiagnosticStatus on_returned_component(std::string component);
	void check_all();

	void sendReport(const diagnostic_msgs::DiagnosticStatus& stat);
	void sendReport(const diagnostic_msgs::DiagnosticArray& stat);

protected:

	ros::Publisher _pub_report;
	ros::Subscriber _sub_heartbeats;

	typedef std::map<std::string, boost::system_time> LastTimeTracker;
	LastTimeTracker _tracker;
};

#endif /* HEARTBEATMONITOR_H_ */
