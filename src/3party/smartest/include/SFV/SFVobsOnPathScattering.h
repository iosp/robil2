/*
 * SFVobsOnPathScattering.h
 *
 *  Created on: Aug 5, 2014
 *      Author: userws3
 */

#ifndef SFVOBSONPATHSCATTERING_H_
#define SFVOBSONPATHSCATTERING_H_


#include <vector>

#include "SFDP/ScenarioFeature.h"
#include "SFV/SFVobstacleOnPath.h"
#include "SFV/SFV.h"


class SFVobsOnPathScattering : public sfvSubGroup  {
private :
	ScenarioFeature * my_num_of_obsOnpath;

	SFVObstacleOnPath * my_obsOnPath_template;
	std::vector <SFVObstacleOnPath *> * my_ObstaclesOnpath;

public :
	SFVobsOnPathScattering(ScenarioFeatureGroup * scenfeaturesGroup,  SFV * parent_SFV);
	SFVobsOnPathScattering(SFVobsOnPathScattering * template_SFVobsOnPathScattering);
	SFVobsOnPathScattering(TiXmlNode * xml_subGroup, SFV * parent_SFV);

	void initFeaturesMap();

	bool roll();
	TiXmlElement * ToXmlElement(int id);

	~SFVobsOnPathScattering();


	inline ScenarioFeature * get_NumberOfObstaclesOnPath()
		{ return(my_num_of_obsOnpath); }

	inline std::vector<SFVObstacleOnPath *> * get_ObstaclesOnPath()
		{ if (! my_ObstaclesOnpath->empty() ) { return my_ObstaclesOnpath; }
		  else { std::cout << "\033[1;31m Obstacles On Path vector is empty \033[0m" << std::endl; return(0); } }

	inline SFVObstacleOnPath * get_ObstaceOnPathTemplate()
		{ return(my_obsOnPath_template); }

};


#endif /* SFVOBSONPATHSCATTERING_H_ */
