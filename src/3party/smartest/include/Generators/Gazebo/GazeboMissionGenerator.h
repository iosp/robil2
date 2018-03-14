/*
 * GazeboMissionGenerator.h
 *
 *  Created on: Apr 10, 2014
 *      Author: userws1
 */

#ifndef GAZEBOMISSIONGENERATOR_H_
#define GAZEBOMISSIONGENERATOR_H_
#include <Generators/GeneratorInterface.h>

class GazeboMissionGenerator : public GeneratorInterface{

public:
	GazeboMissionGenerator();
	virtual ~GazeboMissionGenerator();

	void generateMission(SFV *sfv, std::string fileName);
	void generateMission_ROBIL2(SFV *sfv, std::string fileName);
    void SaharGenerateMissionToOCU(SFV *sfv, std::string fileName);

	void generate(SFV * sfv, std::string scenario_folder_url);

};

#endif /* GAZEBOMISSIONGENERATOR_H_ */
