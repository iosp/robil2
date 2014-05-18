/*
 * ScriptExecuter.cpp
 *
 *  Created on: Oct 17, 2013
 *      Author: blackpc
 */

#include <scriptable_monitor/ScriptExecuter.h>

/*
 * Statics
 */

boost::mutex 				ScriptExecuter::_executionMutex;

PythonScript* 				ScriptExecuter::_currentScript = NULL;
bool 						ScriptExecuter::_runtime;
bool 						ScriptExecuter::_simulation;
bool 						ScriptExecuter::_validationFailed;
vector<string> 				ScriptExecuter::_validationsPassed;
vector<string> 				ScriptExecuter::_validationsFailed;
map<string, string> 		ScriptExecuter::_topicValues;

PyMethodDef ScriptExecuter::MonitorPythonLib[] =
{
	{"internal_function", 	&ScriptExecuter::internalFunction, 		METH_O, ""	},
	{"internal_validation", &ScriptExecuter::internalValidation, 	METH_O, ""	},
	{"internal_topic", 		&ScriptExecuter::internalTopic, 		METH_O, ""	},
	{NULL, 					NULL, 									0, 		NULL}
};

ScriptExecuter::ScriptExecuter()
{
	PythonExecuter::initModule("robil_monitor_lib", MonitorPythonLib);
}

CompilationResult ScriptExecuter::compile(PythonScript& script)
{
	return PythonExecuter::compile(script.getSourceCode());
}

void ScriptExecuter::execute(PythonScript& pythonScript, ScriptInputType input)
{
	boost::mutex::scoped_lock lock(_executionMutex);

	pythonScript.setValidationFailed(false);

	ScriptExecuter::_runtime = true;
	ScriptExecuter::_simulation = false;
	ScriptExecuter::_validationFailed = false;
	ScriptExecuter::_validationsFailed = vector<string>();
	ScriptExecuter::_validationsPassed = vector<string>();
	ScriptExecuter::_currentScript = &pythonScript;
	ScriptExecuter::_topicValues = input;

	PythonExecuter::execute(pythonScript.getSourceCode());
}

void ScriptExecuter::simulate(PythonScript& pythonScript)
{
	boost::mutex::scoped_lock lock(_executionMutex);

	pythonScript.getUsedInternalFunction().clear();
	pythonScript.getUsedTopics().clear();
	pythonScript.setValidationFailed(false);

	ScriptExecuter::_runtime = true;
	ScriptExecuter::_simulation = true;
	ScriptExecuter::_validationFailed = false;
	ScriptExecuter::_validationsFailed.clear();
	ScriptExecuter::_validationsPassed.clear();
	ScriptExecuter::_currentScript = &pythonScript;

	PythonExecuter::execute(pythonScript.getSourceCode());
}

PyObject* ScriptExecuter::convertToPythonType(string object) {
	Yaml yaml = YamlFunctions::toYaml(object);

//	cout << "Converting '" << object << "' to python type" << endl;

	// Empty value
	if (yaml.size() == 0) {
		yaml = YamlFunctions::toYaml("[" + object + "]");
		// return PyFloat_FromDouble(0);
	}

	// Single item
	if (yaml.size() == 1) {

		string value = yaml[0].to<string>();

		// Is boolean
		boost::to_lower(value);
		if (value == "true")
			return PyBool_FromLong(1);
		if (value == "false")
			return PyBool_FromLong(0);


		// Is Double?
		char * p;
		double d = strtod( value.c_str(), &p);

		if ( * p == 0 )
			return PyFloat_FromDouble(d);

		// Must be string
		return PyString_FromString(value.c_str());
	}

	// Array
	if (yaml.size() > 1) {
		PyObject* list = PyList_New(yaml.size());
		for(size_t i = 0; i < yaml.size(); i++)
			PyList_SetItem(list, i, PyString_FromString(yaml[i].to<string>().c_str()));

		return list;
	}

	return PyFloat_FromDouble(0);
}

PyObject* ScriptExecuter::internalTopic(PyObject* self, PyObject* args)
{
	if (_validationFailed)
		return PyString_FromString("0");

	string 	scriptName 	= PyString_AsString(PyTuple_GetItem(args, 0));
	string	topicName 	= PyString_AsString(PyTuple_GetItem(args, 1));

	if (_simulation) {
		_currentScript->addUsedTopic(topicName);
		return PyString_FromString("0");
	}

	string value = _topicValues.count(topicName) > 0 ? _topicValues[topicName].c_str() : "0";
	return convertToPythonType(value);
}

PyObject* ScriptExecuter::internalValidation(PyObject* self, PyObject* args)
{
	if (_validationFailed)
		return PyString_FromString("");

	string 	scriptName 			= PyString_AsString(PyTuple_GetItem(args, 0));
	bool 	validationPassed 	= PyObject_IsTrue(PyTuple_GetItem(args, 1));
	string 	description 		= PyString_AsString(PyTuple_GetItem(args, 2));

	if (_simulation)
		_currentScript->addValidation(description);

	if (!validationPassed) {
		_validationFailed = true;
		_currentScript->setValidationFailed(true);
		_currentScript->addFailedValidation(description);
	}

	return PyString_FromString("");
}

PyObject* ScriptExecuter::internalFunction(PyObject* self, PyObject* args)
{
	if (_validationFailed)
		return PyString_FromString("");

	long int argCount = PyTuple_Size(args);
	string scriptName = PyString_AsString(PyTuple_GetItem(args, 0));
	string functionName = PyString_AsString(PyTuple_GetItem(args, 1));

	vector<string> arguments;
	for (int i = 2; i < argCount; i++) {
		string argument = PyString_AsString(PyTuple_GetItem(args, i));
		arguments.push_back(argument);
	}

	if (_simulation) {
		_currentScript->addUsedInternalFunction(functionName);
		return PyString_FromString("0");
	} else {

		InternalFunction* function = InternalFunctionsManager::resolve(functionName);

		if (function == NULL)
			return PyFloat_FromDouble(0);

		Parameters input(arguments);
		Parameters output = function->invoke(input);

		if (output.size() == 1) {
			return convertToPythonType("[" + output.get<string>(0) + "]");
		}

		if (output.size() > 1) {
			PyObject* list = PyList_New(output.size());

			for(size_t i = 0; i < output.size(); i++)
				PyList_SetItem(list, i, convertToPythonType("[" + output.get<string>(i) + "]"));

			return list;
		}

		return PyFloat_FromDouble(0.9);
	}
}
