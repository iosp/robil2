/*
 * scriptable_analyzer.h
 *
 *  Created on: Oct 17, 2013
 *      Author: blackpc
 */

#ifndef SCRIPTABLE_ANALYZER_H_
#define SCRIPTABLE_ANALYZER_H_

#include <iostream>
#include <ros/ros.h>
#include <diagnostic_aggregator/analyzer.h>
#include <diagnostic_aggregator/status_item.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <pluginlib/class_list_macros.h>

#include <scriptable_monitor/ScriptableMonitorNode.h>

using namespace std;

namespace diagnostic_aggregator {

class ScriptableMonitor : public Analyzer {
public:
	ScriptableMonitor();
	virtual ~ScriptableMonitor();

	bool init(const string baseName, const ros::NodeHandle& nodeHandle);
	bool match(const string name);
	bool analyze(const boost::shared_ptr<StatusItem> item);
	vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > report();
	string getPath() const { return "scriptable_monitor"; }
	string getName() const { return "Scriptable Monitor"; }

private:
	ScriptableMonitorNode* _scriptNode;
};

}

#endif /* SCRIPTABLE_ANALYZER_H_ */
