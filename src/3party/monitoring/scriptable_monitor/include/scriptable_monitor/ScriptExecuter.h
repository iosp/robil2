/*
 * ScriptExecuter.h
 *
 *  Created on: Oct 17, 2013
 *      Author: blackpc
 */

#ifndef ScriptExecuter_H_
#define ScriptExecuter_H_

#include <python2.7/Python.h>
#include <ros/ros.h>
#include <iostream>
#include <map>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include <scriptable_monitor/YamlFunctions.h>
#include <scriptable_monitor/InternalFunction.h>

#include "PythonScript.h"
#include "PythonExecuter.h"

#define foreach BOOST_FOREACH

typedef map<string, string> ScriptInputType;

using namespace std;

class ScriptExecuter
{
public:
	ScriptExecuter();

	/**
	 *
	 * @param sourceCode
	 * @return
	 */
	CompilationResult compile(PythonScript& script);
	void execute(PythonScript& pythonScript, ScriptInputType input);

	/**
	 * Executes python script without validations and internal function calls.
	 * Used to extract involved topic names and validation names
	 * @param pythonScript
	 */
	void simulate(PythonScript& pythonScript);

private:

	// //////////////////////////////////////////////////////////////////
	// // Execution context /////////////////////////////////////////////
	// //////////////////////////////////////////////////////////////////

	static boost::mutex _executionMutex;
	static PythonScript* _currentScript;

	static bool _runtime;
	static bool _simulation;
	static bool _validationFailed;
	static vector<string> _validationsPassed;
	static vector<string> _validationsFailed;
	static map<string, string> _topicValues;
	static PyMethodDef MonitorPythonLib[];

	static PyObject* convertToPythonType(string value);

	static PyObject* internalTopic(PyObject* self, PyObject* args);
	static PyObject* internalValidation(PyObject* self, PyObject* args);
	static PyObject* internalFunction(PyObject* self, PyObject* args);

};


#endif /* ScriptExecuter_H_ */
