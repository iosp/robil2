/*
 * scenarioLauncher.h
 *
 *  Created on: Aug 25, 2014
 *      Author: userws3
 */

#ifndef SCENARIOLAUNCHER_H_
#define SCENARIOLAUNCHER_H_

#include <Python.h>
#include <string>

class ScenarioLauncher
{

	private:
		PyObject *pName, *pModule, *pDict, *pClass;
		PyObject *pInstance;

	public:
		ScenarioLauncher(std::string pyInterface);
		~ScenarioLauncher();

		void start_launcher();
		void stop_launcher();

		void startGazeboServer(std::string Scenarin_folder);
		void launchGazeboClient();
		void setScenarioEnv(std::string Scenarin_folder);
		void launchPlatformControlsSpawner();
		void launchPlatformControlsUnspawner();

		void launchWPdriver(std::string Scenarin_folder);
		void launchTFbroadcaster();
		void launchRecorder(std::string Scenarin_folder);
		void launchGrader(std::string Scenarin_folder);

		void GazeboPause();
		void GazeboUnPause();
};

#endif /* SCENARIOLAUNCHER_H_ */
