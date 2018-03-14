/*
 * scenarioLauncher.cpp
 *
 *  Created on: Aug 25, 2014
 *      Author: userws3
 */


#include "Executor/scenarioLauncher.h"
#include <iostream>
#include <Python.h>
#include <string>

ScenarioLauncher::ScenarioLauncher(std::string pyInterface)
{
	Py_Initialize();

	char *g="";
	PySys_SetArgv(1, &g);

	pName = PyString_FromString(pyInterface.c_str());	// Build the name object

	pModule = PyImport_Import(pName);             // Load the module object
	if ( ! pModule)
		{
			PyErr_Print();
			std::cout << "\033[1;31m Fail to load Python coordinator module \033[0m" << std::endl;
		}


	pDict = PyModule_GetDict(pModule);	// pDict is a borrowed reference
	if ( ! pDict)
		{
			PyErr_Print();
			std::cout << "\033[1;31m Fail to load Python coordinator module \033[0m" << std::endl;
		}


	pClass = PyDict_GetItemString(pDict, "ScenarioLauncher");  // Build the name of a callable class
	if ( ! pClass)
		{
			PyErr_Print();
			std::cout << "\033[1;31m Fail to load Python SimCordinator class \033[0m" << std::endl;
		}


	if (PyCallable_Check(pClass))
	{
	  pInstance = PyObject_CallObject(pClass, NULL); // Create an instance of the class
	}
}

void ScenarioLauncher::startGazeboServer(std::string Scenarin_folder)
{
	PyObject_CallMethod(pInstance, "runGazeboServer", "(s)", Scenarin_folder.c_str() ,NULL);
}

void ScenarioLauncher::launchGazeboClient()
{
	PyObject_CallMethod(pInstance, "runGazeboClient",  NULL);
}


void ScenarioLauncher::setScenarioEnv(std::string Scenarin_folder)
{
	PyObject_CallMethod(pInstance, "setScenarioEnv", "(s)", Scenarin_folder.c_str() , NULL);
}

void ScenarioLauncher::start_launcher()
{
	PyObject_CallMethod(pInstance, "start_launcher", NULL);
}

void ScenarioLauncher::stop_launcher()
{
	PyObject_CallMethod(pInstance, "stop_launcher", NULL);
}

void ScenarioLauncher::launchPlatformControlsSpawner()
{
	PyObject_CallMethod(pInstance, "launch_platform_controls_spawner", NULL);
}

void ScenarioLauncher::launchPlatformControlsUnspawner()
{
	PyObject_CallMethod(pInstance, "launch_platform_controls_unspawner", NULL);
}

void ScenarioLauncher::launchWPdriver(std::string Scenarin_folder)
{
	PyObject_CallMethod(pInstance, "launch_WPdriver", "(s)", Scenarin_folder.c_str() , NULL);
}

void ScenarioLauncher::launchTFbroadcaster()
{
	PyObject_CallMethod(pInstance, "launch_robot_tf_broadcaster", NULL);
}

void ScenarioLauncher::launchRecorder(std::string Scenarin_folder)
{
	PyObject_CallMethod(pInstance, "launch_recorder", "(s)", Scenarin_folder.c_str() , NULL);
}

void ScenarioLauncher::launchGrader(std::string Scenarin_folder)
{
	PyObject_CallMethod(pInstance, "launch_grader", "(s)", Scenarin_folder.c_str() , NULL);
}

void ScenarioLauncher::GazeboPause()
{
	PyObject_CallMethod(pInstance, "Gazebo_Pause", NULL);
}

void ScenarioLauncher::GazeboUnPause()
{
	PyObject_CallMethod(pInstance, "Gazebo_UnPause", NULL);
}


ScenarioLauncher::~ScenarioLauncher()
{
	Py_DECREF(pClass);
	Py_DECREF(pDict);
	Py_DECREF(pModule);
	Py_DECREF(pName);
	Py_DECREF(pInstance);
	Py_Finalize();
}
