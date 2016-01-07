/*
 * ScriptHost.h
 *
 *  Created on: Oct 27, 2013
 *      Author: blackpc
 */

#ifndef SCRIPTHOST_H_
#define SCRIPTHOST_H_

#include <iostream>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

#include <scriptable_monitor/InternalFunction.h>
#include <scriptable_monitor/PythonScript.h>
#include <scriptable_monitor/PredicatesScript.h>
#include <scriptable_monitor/ScriptExecuter.h>
#include <scriptable_monitor/RosTopicListener.h>

#define foreach BOOST_FOREACH

using namespace std;

typedef boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> DiagnosticStatusPtr;
//#define lock(mutexObject) boost::mutex::scoped_lock mutexObject##Lock(mutexObject)

struct AddScriptResponse {
	bool success;
	string message;
};

class ScriptHost
{
public:
	ScriptHost();
	virtual ~ScriptHost();

	void start();
	void stop();

	AddScriptResponse addScript(string sourceCode);
	vector<DiagnosticStatusPtr> getDiagnosticStatusesAndClear();
	set<PythonScriptPtr> getScripts();
	void deleteScript(string scriptName);
	void pauseModule(string scriptName);
	void resumeModule(string scriptName);

private:

	/**
	 * Scripts execution check interval in seconds
	 */
	double _executionInterval;
	boost::shared_ptr<boost::thread> _workThread;
	set<PythonScriptPtr> _scripts;
	ScriptExecuter _executer;

	boost::recursive_mutex _scriptsMutex;
	boost::recursive_mutex _statusesMutex;
	vector<DiagnosticStatusPtr> _diagnosticStatuses;

	void run();
	bool prepareScript(PythonScript& script);
	bool isExecutionTime(PythonScriptPtr script);
	bool hasAllTopicValues(PythonScriptPtr script);
	void addDiagnosticStatus(PythonScriptPtr script);
	void addDiagnosticStatus(string name, string hid, int8_t level, string message);
	void checkTimers();

	PythonScriptPtr getScript(string scriptName);
	bool scriptExists(string scriptName);


	bool typeAnalization(string sourceCode, AddScriptResponse& response);
	bool typeAnalizationForRemove(string sourceCode);
	AddScriptResponse addScript(string sourceCode, AddScriptResponse& response);
};

#endif /* SCRIPTHOST_H_ */
