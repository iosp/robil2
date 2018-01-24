/*
 * GazeboScenarioGenerator.h
 *
 *  Created on: Jun 30, 2014
 *      Author: userws3
 */

#ifndef GAZEBOSCENARIOGENERATOR_H_
#define GAZEBOSCENARIOGENERATOR_H_

#include <Generators/GeneratorInterface.h>
#include "SFV/SFV.h"


class GazeboScenarioGenerator
{
	private :
		SFV* my_sfv;
		std::string my_scenario_folder_url;

		std::vector<GeneratorInterface *> * generators_vec;

	public:

		GazeboScenarioGenerator(SFV* sfv, std::string scenario_folder_url);
		inline virtual ~GazeboScenarioGenerator() {};

		int GenerateScenario();
};



#endif /* GAZEBOSCENARIOGENERATOR_H_ */
