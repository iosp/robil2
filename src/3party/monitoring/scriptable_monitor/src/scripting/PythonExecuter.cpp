/*
 * PythonExecuter.cpp
 *
 *  Created on: Oct 28, 2013
 *      Author: blackpc
 */

#include <scriptable_monitor/PythonExecuter.h>

PyThreadState* 	PythonExecuter::_mainState = NULL;
set<string>		PythonExecuter::_initializedModules;

void PythonExecuter::initialize()
{
	if (Py_IsInitialized())
		return;

	int argc = 1;
	char* argv[] = { "python_executer" };
	PyEval_InitThreads();
	Py_InitializeEx(0);
	PyObject *pModule = PyImport_AddModule("__main__");
	PySys_SetArgv(argc, argv);
	_mainState = PyEval_SaveThread();
}

void PythonExecuter::finalize()
{
	if (!Py_IsInitialized())
		return;

	PyEval_RestoreThread(_mainState);
	Py_Finalize();
}

void PythonExecuter::initModule(string name, PyMethodDef* methodDef)
{
	if (!Py_IsInitialized())
		initialize();

	if (_initializedModules.count(name) > 0) {
		cout << "Module '" << name << "' already added to python environment" << endl;
		return;
	}

	_initializedModules.insert(name);

	PyEval_RestoreThread(_mainState);
	Py_InitModule(name.c_str(), methodDef);
	_mainState = PyEval_SaveThread();

	cout << "Module '" << name << "' successfully added to python environment" << endl;
}

void PythonExecuter::execute(string code)
{
	if (!Py_IsInitialized())
		initialize();

	PyGILState_STATE state = PyGILState_Ensure();
	PyRun_SimpleString(code.c_str());
	PyGILState_Release(state);
}

CompilationResult PythonExecuter::compile(string code)
{
	if (!Py_IsInitialized())
		initialize();

	PyGILState_STATE state = PyGILState_Ensure();

	CompilationResult result;
	PyObject* codeObject = Py_CompileString(code.c_str(), "", Py_file_input);
	result.success = codeObject != NULL;

	PyGILState_Release(state);

	return result;
}
