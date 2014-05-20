/*
 * PredicatesScript.cpp
 *
 *  Created on: Oct 20, 2013
 *      Author: blackpc
 */

#include <scriptable_monitor/PredicatesScript.h>

PredicatesScript::PredicatesScript(string script, set<string>& internalFunctions)
{
	_internalFunctions = internalFunctions;
	_sourceScript = script;
	_translatedScript = script;

	try {
		processParameters(_translatedScript);

		if (getParameter("type") == "predicate") {
			processTopics(_translatedScript);
			processInternalFunctions(_translatedScript);
			processPredicates(_translatedScript);
		}
	} catch(...) {

	}
}

bool PredicatesScript::addParameter(
		const boost::match_results<std::string::const_iterator>& paramMatch)
{
	string paramName = paramMatch[1].str();
	string paramValue = paramMatch[2].str();

	if (paramName == "name")
		_name = paramValue;
	else {
		_parameters[paramName] = paramValue;
	}

	return true;
}

void PredicatesScript::processParameters(string& script)
{
	boost::regex regex("\\#\\!\\s([a-zA-Z_]+)\\s([^\\n]+)", boost::regbase::normal | boost::regbase::icase);
	boost::sregex_iterator m1(script.begin(), script.end(), regex);
	boost::sregex_iterator m2;
	std::for_each(m1, m2, boost::bind(&PredicatesScript::addParameter, this, _1));
}

void PredicatesScript::processTopics(string& script)
{
	boost::regex regex("\\{(([a-zA-Z0-9_/]+(\\[[0-9\\:]+\\])?)+)\\}", boost::regbase::normal | boost::regbase::icase);
	script = boost::regex_replace(script, regex, "topic(\"\\1\")");
}

void PredicatesScript::processPredicates(string& script)
{
	using namespace boost::algorithm;

	vector<string> lines;
	vector<string> processedLines;
	split(lines, script, is_any_of("\n"));

	foreach(string line, lines) {
		if (contains(line, ">") || contains(line, "<") || contains(line, "==") || contains(line, "!=")) {
			string escapedLine = line;
			boost::replace_all(escapedLine, "\"", "'");
			string newLine = "validate.is_true(" + line + ", \"" + escapedLine + "\")";
			processedLines.push_back(newLine);
		} else
			processedLines.push_back(line);
	}

	script = join(processedLines, "\n");
}

void PredicatesScript::processInternalFunctions(string& script)
{
	using namespace boost::algorithm;

	vector<string> lines;
	vector<string> processedLines;
	split(lines, script, is_any_of("\n"));

	foreach(string line, lines) {
		int bracketDepth = 0;
		bool func = false;
		bool withinQuotes = false;
		int funcStart = 0;
		int funcEnd = 0;
		string funcName = "";

		for (int i = 0; i < line.size() + 1; i++) {

			char currentChar;

			if (i < line.size())
				currentChar = line[i];
			else
				currentChar = ' ';


			// Quotes check
			if (!withinQuotes && (currentChar == '"' || currentChar == '\'')) {
				withinQuotes = true;
				continue;
			} else if (withinQuotes && (currentChar == '"' || currentChar == '\'')) {
				withinQuotes = false;
				continue;
			}

			// Brackets
			if (currentChar == '(') {

				if (bracketDepth == 0 && funcName.size() > 0) {
					func = true;
					funcStart = i - funcName.length();
				}

				bracketDepth++;
				continue;
			} else if (currentChar == ')') {
				bracketDepth--;

				if (func && bracketDepth == 0) {
					funcEnd = i;
					break;
				}
				continue;
			}

			// Function name
			if (bracketDepth == 0) {
				if (isalnum(currentChar) || currentChar == '_') {
					funcName += currentChar;
					continue;
				} else {
					funcName = "";
				}
			}

		}

		if (_internalFunctions.count(funcName) > 0) {
			string parameters = line.substr(funcStart + funcName.length() + 1, funcEnd - funcStart - 1 - funcName.length() );
			string newLine = line.substr(0, funcStart) + "robil_lib.invoke('" + funcName + "'" + ( parameters.size() > 0 ? ", " + parameters : "") + ")";
			processedLines.push_back(newLine);
		} else {
			processedLines.push_back(line);
		}


	}

	script = join(processedLines, "\n");
}

