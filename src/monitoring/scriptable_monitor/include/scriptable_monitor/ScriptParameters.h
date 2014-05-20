/*
 * ScriptParameters.h
 *
 *  Created on: Oct 20, 2013
 *      Author: blackpc, danerde@gmail.com
 */

#ifndef ScriptParameters_H_
#define ScriptParameters_H_

#include <iostream>
#include <boost/regex.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include "PythonScript.h"

#define foreach BOOST_FOREACH

using namespace std;

class ScriptParameters
{
public:
	/**
	 * Parses provided predicates script to python code
	 * @param script The script
	 * @param internalFunctions Internal known function which will be replaced with appropriate wrapper
	 */
	ScriptParameters(string script);

	/**
	 * Returns the source script without changes
	 * @return
	 */
	string 			getSourceScript() 		{ return _sourceScript; }

	/**
	 * Returns name of the script
	 * @return
	 */
	string 			getName()				{ return _name; }

	/**
	 * Returns parameter value by name
	 * @param name Parameter name
	 * @return
	 */
	string getParameter(string name) {
		return (_parameters.count(name) > 0) ? _parameters[name] : "";
	}

	string operator[](string name){ return getParameter(name); }

private:

	/**
	 * Extracts all parameters from script (#! param_name param_value)
	 * @param script
	 */
	void processParameters(string& script);

	/**
	 * Callback method for regex param search
	 * @param paramMatch
	 * @return
	 */
	bool addParameter(const boost::match_results<std::string::const_iterator>& paramMatch);

	string _sourceScript;
	string _name;
	map<string, string> _parameters;
};

#endif /* ScriptParameters_H_ */
