
#include "Generators/Gazebo/GazeboScenarioGenerator.h"

//#include "Generators/Gazebo/GazeboDisruptionGenerator.h"
#include "Generators/Gazebo/GazeboEnvironmentGenerator.h"
#include "Generators/Gazebo/GazeboPlatformGenerator.h"
#include "Generators/Gazebo/GazeboMissionGenerator.h"

#include "Generators/GeneratorInterface.h"

#include <string>
#include <iostream>

GazeboScenarioGenerator::GazeboScenarioGenerator(SFV* sfv, std::string scenario_folder_url)
{
	my_sfv = sfv;
	my_scenario_folder_url = scenario_folder_url;

	generators_vec = new std::vector<GeneratorInterface *>;

	generators_vec->push_back(new GazeboMissionGenerator);
	generators_vec->push_back(new GazeboPlatformGenerator);
	generators_vec->push_back(new GazeboEnvironmentGenerator);
}

int GazeboScenarioGenerator::GenerateScenario()
{
      for(auto gen_it : * generators_vec)
      {
    	  	  gen_it->generate(my_sfv, my_scenario_folder_url);
      }
      return(1);
}



