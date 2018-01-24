/*
 * EnvironmentGenerator.h
 *
 *  Created on: Feb 25, 2014
 *      Author: userws1
 */

#ifndef GAZEBOENVIRONMENTGENERATOR_H_
#define GAZEBOENVIRONMENTGENERATOR_H_

#include <string>
#include <utils/TinyXmlDef.h>
#include <Generators/GeneratorInterface.h>
#include <Generators/Gazebo/TerrainAnalyzer.h>

class GazeboEnvironmentGenerator : public GeneratorInterface {
	TerrainAnalyzer* m_terrainAnalyzer;
	int m_objectCount;
	int m_ObstacleOnPathCounter;
	int m_WpMarkCounter;

	void spawnObjects(SFV* sfv, TiXmlElement * element);

	void spawnObstacleOnPath(SFV* sfv,TiXmlElement * element);


	void spawnTerrain(SFV* sfv,TiXmlElement * element);
	void spawnPlatformPose(SFV* sfv,TiXmlElement * element);
	void spawnPathWpMarks(SFV* sfv,TiXmlElement * element);



public:

	GazeboEnvironmentGenerator();
	virtual ~GazeboEnvironmentGenerator();

	void genEnvFromSFV(SFV * sfv, std::string filename);
	void generate(SFV * sfv, std::string scenario_folder_url);
};

#endif /* GAZEBOENVIRONMENTGENERATOR_H_ */
