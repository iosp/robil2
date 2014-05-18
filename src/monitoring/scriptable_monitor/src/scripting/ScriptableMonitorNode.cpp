/*
 * ScriptableAnalyzerNode.cpp
 *
 *  Created on: Oct 30, 2013
 *      Author: blackpc
 */

#include <scriptable_monitor/ScriptableMonitorNode.h>

ScriptableMonitorNode::ScriptableMonitorNode()
{
	_scriptHost.start();

	ros::NodeHandle node("~");

	_addScriptSubscriber =
			node.subscribe("/scriptable_monitor/add_script", 0, &ScriptableMonitorNode::onAddScriptMessage, this);

	_deleteScriptSubscriber =
			node.subscribe("/scriptable_monitor/delete_script", 0, &ScriptableMonitorNode::onDeleteScriptMessage, this);

	_getScriptsService =
			node.advertiseService("/scriptable_monitor/get_scripts", &ScriptableMonitorNode::getScripts, this);

	_addScriptService =
			node.advertiseService("/scriptable_monitor/add_script", &ScriptableMonitorNode::addScript, this);
}

ScriptableMonitorNode::~ScriptableMonitorNode()
{
	_addScriptSubscriber.shutdown();
	_deleteScriptSubscriber.shutdown();
	_scriptHost.stop();
}

bool ScriptableMonitorNode::addScript(
		scriptable_monitor::AddScriptRequest& request,
		scriptable_monitor::AddScriptResponse& response)
{
	cout << "================================================" << endl;
	cout << request.script.data << endl;
	cout << "================================================" << endl;

	AddScriptResponse addScriptResponse = _scriptHost.addScript(request.script.data);

	response.message.data = addScriptResponse.message;
	response.success = addScriptResponse.success ? 1 : 0;

	return true;
}

bool ScriptableMonitorNode::getScripts(
		scriptable_monitor::GetScriptsRequest& request,
		scriptable_monitor::GetScriptsResponse& response)
{
	set<PythonScriptPtr> scripts = _scriptHost.getScripts();

	foreach(PythonScriptPtr script, scripts) {
		scriptable_monitor::Script s;

		s.name = script->getName();
		s.interval = script->getInterval();
		s.validations = script->getValidations();
		s.type = "Python";
		s.fail_type = (int)script->getFailType();
		// s.last_execution = script->getExecutionTime

		foreach(string topic, script->getUsedTopics())
			s.topics.push_back(topic);

		foreach(string function, script->getUsedInternalFunction())
			s.functions.push_back(function);

		response.scripts.push_back(s);
	}

	return true;
}

void ScriptableMonitorNode::onAddScriptMessage(
		const std_msgs::String::ConstPtr script)
{
	_scriptHost.addScript(script->data);
}

void ScriptableMonitorNode::onDeleteScriptMessage(
		const std_msgs::String::ConstPtr scriptName)
{
	_scriptHost.deleteScript(scriptName->data);
}
