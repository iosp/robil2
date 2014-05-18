/*
 * PythonExecuter.h
 *
 *  Created on: Oct 28, 2013
 *      Author: blackpc
 */

#ifndef PYTHONEXECUTER_H_
#define PYTHONEXECUTER_H_

#include <iostream>
#include <set>
#include <boost/lexical_cast.hpp>
#include <python2.7/Python.h>

using namespace std;

struct CompilationResult {
	bool 	success;
	string 	message;

	CompilationResult() { message = ""; success = false; }
};

class PythonExecuter
{

public:

	static void initialize();

	static void initModule(string name, PyMethodDef* methodDef);
	static void execute(string code);
	static CompilationResult compile(string code);

	static void finalize();

private:

	static PyThreadState* 	_mainState;
	static set<string>		_initializedModules;

	PythonExecuter() { }

};

#endif /* PYTHONEXECUTER_H_ */
