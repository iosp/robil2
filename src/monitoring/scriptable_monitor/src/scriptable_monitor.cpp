/*
 * scriptable_analyzer.cpp
 *
 *  Created on: Oct 17, 2013
 *      Author: blackpc
 */

#include <scriptable_monitor/scriptable_monitor.h>

PLUGINLIB_REGISTER_CLASS(ScriptableMonitor,
                         diagnostic_aggregator::ScriptableMonitor,
                         diagnostic_aggregator::Analyzer)

namespace diagnostic_aggregator {

ScriptableMonitor::ScriptableMonitor() {
	_scriptNode = new ScriptableMonitorNode();
}

ScriptableMonitor::~ScriptableMonitor() {
	delete _scriptNode;
}

bool ScriptableMonitor::init(const string baseName,
		const ros::NodeHandle& nodeHandle)
{
	return true;
}

bool ScriptableMonitor::match(const string name)
{
	return true;
}

bool ScriptableMonitor::analyze(const boost::shared_ptr<StatusItem> item)
{
	return true;
}

vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > ScriptableMonitor::report()
{
	return _scriptNode->getScriptHost().getDiagnosticStatusesAndClear();
}

}
