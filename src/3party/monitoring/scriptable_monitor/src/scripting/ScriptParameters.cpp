/*
 * ScriptParameters.cpp
 *
 *  Created on: Oct 20, 2013
 *      Author: blackpc, danerde@gmail.com
 */

#include <scriptable_monitor/ScriptParameters.h>

ScriptParameters::ScriptParameters(string script)
{
	_sourceScript = script;

	try {
		processParameters(_sourceScript);
	} catch(...) {

	}
}

bool ScriptParameters::addParameter(
		const boost::match_results<std::string::const_iterator>& paramMatch)
{
	string paramName = paramMatch[1].str();
	string paramValue = paramMatch[2].str();

	if (paramName == "name")
		_name = paramValue;
	//else
	{
		_parameters[paramName] = paramValue;
	}

	return true;
}

void ScriptParameters::processParameters(string& script)
{
	boost::regex regex("\\#\\!\\s([a-zA-Z_]+)\\s([^\\n]+)", boost::regbase::normal | boost::regbase::icase);
	boost::sregex_iterator m1(script.begin(), script.end(), regex);
	boost::sregex_iterator m2;
	std::for_each(m1, m2, boost::bind(&ScriptParameters::addParameter, this, _1));
}

