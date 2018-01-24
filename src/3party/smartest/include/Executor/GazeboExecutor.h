/*
 * GazeboExecutor.h
 *
 *  Created on: Jul 7, 2014
 *      Author: userws3
 */

#ifndef GAZEBOEXECUTOR_H_
#define GAZEBOEXECUTOR_H_

#include <string>
#include "Executor/scenarioLauncher.h"
#include "SFV/SFV.h"
#include "std_msgs/Float32MultiArray.h"


class GazeboExecutor {
	private :

		std::string my_Scenario_folder_url;

		std::string my_pyInterface;

		std_msgs::Float32MultiArray::ConstPtr  my_scenario_graedes;
		bool was_executed_flag;

		ScenarioLauncher *my_launcher;

	public :
		GazeboExecutor(SFV *sfv);
		~GazeboExecutor();

		int RunScenario(int argc, char** argv);

		inline std_msgs::Float32MultiArray::ConstPtr get_scenario_grades()
			{	return my_scenario_graedes; }

		inline bool get_wasExecutedFlag()
			{	return was_executed_flag; }
};



#endif /* GAZEBOEXECUTOR_H_ */
