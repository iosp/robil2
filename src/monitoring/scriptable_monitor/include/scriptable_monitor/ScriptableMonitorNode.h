/*
 * ScriptableAnalyzerNode.h
 *
 *  Created on: Oct 30, 2013
 *      Author: blackpc
 */

#ifndef SCRIPTABLEANALYZERNODE_H_
#define SCRIPTABLEANALYZERNODE_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <scriptable_monitor/ScriptHost.h>

#include <scriptable_monitor/Script.h>
#include <scriptable_monitor/GetScripts.h>
#include <scriptable_monitor/AddScript.h>


class ScriptableMonitorNode
{
public:

	ScriptableMonitorNode();
	~ScriptableMonitorNode();

	ScriptHost& getScriptHost() { return _scriptHost; }

private:
	ScriptHost _scriptHost;
	ros::Subscriber _addScriptSubscriber;
	ros::Subscriber _deleteScriptSubscriber;
	ros::ServiceServer _getScriptsService;
	ros::ServiceServer _addScriptService;

	bool addScript(scriptable_monitor::AddScriptRequest& request, scriptable_monitor::AddScriptResponse& response);
	bool getScripts(scriptable_monitor::GetScriptsRequest& request, scriptable_monitor::GetScriptsResponse& response);

	void onAddScriptMessage(const std_msgs::String::ConstPtr script);
	void onDeleteScriptMessage(const std_msgs::String::ConstPtr scriptName);
};

#endif /* SCRIPTABLEANALYZERNODE_H_ */
