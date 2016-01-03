/*
 * PredicatesScript.h
 *
 *  Created on: Oct 20, 2013
 *      Author: blackpc
 */

#ifndef PREDICATESSCRIPT_H_
#define PREDICATESSCRIPT_H_

#include <iostream>
#include <boost/regex.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include "PythonScript.h"

#define foreach BOOST_FOREACH

using namespace std;

class PredicatesScript
{
public:
	/**
	 * Parses provided predicates script to python code
	 * @param script The script
	 * @param internalFunctions Internal known function which will be replaced with appropriate wrapper
	 */
	PredicatesScript(string script, set<string>& internalFunctions);

	/**
	 * Returns the source script without changes
	 * @return
	 */
	string 			getSourceScript() 		{ return _sourceScript; }

	/**
	 * Returns translated python script
	 * @return
	 */
	string 			getTranslatedScript()	{ return _translatedScript; }

	/**
	 * Returns name of the script
	 * @return
	 */
	string 			getName()				{ return _name; }

	/**
	 * Returns translated python script
	 * @return
	 */
	PythonScript	getPythonScript() 		{ return PythonScript(_name, getTranslatedScript(), _parameters); }

	/**
	 * Returns parameter value by name
	 * @param name Parameter name
	 * @return
	 */
	string getParameter(string name) {
		return (_parameters.count(name) > 0) ? _parameters[name] : "";
	}

	/**
	 * Adds a name of internal function
	 * @param functionName
	 */
	void addInternalFunction(string functionName) { if (_internalFunctions.count(functionName) == 0) _internalFunctions.insert(functionName); }

private:

	/**
	 * Extracts all parameters from script (#! param_name param_value)
	 * @param script
	 */
	void processParameters(string& script);

	/**
	 * Replaces all ros topics {topic_name} with python method topic('topic_name')
	 * @param script
	 * @return
	 */
	void processTopics(string& script);

	/**
	 * Replaces all predicates with validate.is_true(predicate)
	 * @param script
	 */
	void processPredicates(string& script);

	/**
	 * Replaces call to known function with python wrapper (robil_lib.invoke('function_name', param1, param2, ...)
	 * @param script
	 */
	void processInternalFunctions(string& script);

	/**
	 * Callback method for regex param search
	 * @param paramMatch
	 * @return
	 */
	bool addParameter(const boost::match_results<std::string::const_iterator>& paramMatch);

	string _sourceScript;
	string _translatedScript;
	string _name;
	map<string, string> _parameters;
	set<string> _internalFunctions;
};

#endif /* PREDICATESSCRIPT_H_ */
