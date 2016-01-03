/*
 * PythonScript.h
 *
 *  Created on: Oct 17, 2013
 *      Author: blackpc
 */

#ifndef PYTHONSCRIPT_H_
#define PYTHONSCRIPT_H_

#include <iostream>
#include <boost/date_time.hpp>

#include <diagnostic_aggregator/status_item.h>
#include <scriptable_monitor/scripts/PythonScriptWrapper.h>

using namespace std;

typedef vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > ScriptOutputType;

class PythonScript {
public:

	enum FailType {
		OK    = 0,
		WARN  = 1,
		ERROR = 2,
		STALL = 3
	};

	PythonScript(string name, string sourceCode, map<string, string> parameters) {
		init(name, sourceCode, parameters);
	}

	PythonScript(string name, string sourceCode) {
		init(name, sourceCode, map<string, string>());
	}

	PythonScript(const PythonScript& script) { *this = script; }

	void addOutput(boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> status) {
		_output.push_back(status);
	}

	string getSourceCode() { return _sourceCode; }

	ScriptOutputType getOutput() { return _output; }

	void addUsedTopic(string topicName) {
		if (_usedTopics.count(topicName) == 0)
			_usedTopics.insert(topicName);
	}

	set<string>& getUsedTopics() { return _usedTopics; }

	void addUsedInternalFunction(string functionName) {
		if (_usedInternalFunctions.count(functionName) == 0)
			_usedInternalFunctions.insert(functionName);
	}

	set<string>& getUsedInternalFunction() { return _usedInternalFunctions; }

	void setValidationFailed(bool isFailed) { _validationFailed = isFailed; }
	void addValidation(string description) { _validations.push_back(description); }
	void addFailedValidation(string description) { _failedValidations.push_back(description); }
	bool isValidationFailed() { return _validationFailed; }
	string getFailedValidation() { return _failedValidations.size() > 0 ? _failedValidations[0] : ""; }
	vector<string>& getValidations() { return _validations; }

	void updateExecutionTime() { _executionTime = boost::posix_time::second_clock::local_time(); }
	void updateExecutionTime(boost::posix_time::ptime executionTime) { _executionTime = executionTime; }
	boost::posix_time::ptime getExecutionTime() { return _executionTime; }

	double getInterval() { return _interval; }

	FailType getFailType() { return _failType; }

	string getParameter(string name) { return _parameters.count(name) > 0 ? _parameters[name] : ""; }

	string getName() { return _name; }

	bool operator<(const PythonScript& script) const {
		return strcmp(script._name.c_str(), this->_name.c_str()) > 0;
	}

private:
	string _name;
	string _sourceCode;
	bool   _validationFailed;

	set<string> _usedTopics;
	set<string> _usedInternalFunctions;
	vector<string> _validations;
	vector<string> _failedValidations;
	map<string, string> _parameters;
	boost::posix_time::ptime _executionTime;
	FailType _failType;

	/**
	 * Execution interval in seconds
	 */
	double _interval;

	ScriptOutputType _output;

	void init(string name, string sourceCode, map<string, string> parameters) {
		_name = name;
		_sourceCode = string(PYTHON_SCRIPT_WRAPPER) + sourceCode;
		_validationFailed = false;
		_parameters = parameters;
		_executionTime = boost::posix_time::microsec_clock::local_time();

		_failType = PythonScript::ERROR;
		if (parameters.count("fail") > 0) {
			string failType = parameters["fail"];
			boost::to_lower(failType);

			if (failType == "ok")
				_failType = PythonScript::OK;

			if (failType == "warn" || failType == "warning")
				_failType = PythonScript::WARN;

			if (failType == "error")
				_failType = PythonScript::ERROR;

			if (failType == "stall")
				_failType = PythonScript::STALL;
		}

		_interval = 1;
		try { _interval = boost::lexical_cast<double>(parameters.count("interval") > 0 ? parameters["interval"] : "1"); }
		catch(...) { }

	}
};

typedef boost::shared_ptr<PythonScript> PythonScriptPtr;

#endif /* PYTHONSCRIPT_H_ */
