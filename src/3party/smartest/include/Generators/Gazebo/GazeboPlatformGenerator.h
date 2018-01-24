/*
 * GazeboPlatformGenerator.h
 *
 *  Created on: Mar 27, 2014
 *      Author: userws1
 */

#ifndef GAZEBOPLATFORMGENERATOR_H_
#define GAZEBOPLATFORMGENERATOR_H_

#include <Generators/GeneratorInterface.h>

class GazeboPlatformGenerator : public GeneratorInterface{
public:
	GazeboPlatformGenerator();
	virtual ~GazeboPlatformGenerator();
	void generatePlatform(SFV * sfv,std::string filename, std::string resources_file_path,  std::string scenario_platform_model_folder_url);
	void generate(SFV * sfv, std::string scenario_folder_url);
};

#endif /* GAZEBOPLATFORMGENERATOR_H_ */
