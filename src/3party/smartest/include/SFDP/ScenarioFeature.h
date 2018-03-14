/*
 * ScenarioFeature.h
 *
 *  Created on: Feb 3, 2014
 *      Author: userws1
 */

#ifndef SCENARIOFEATURE_H_
#define SCENARIOFEATURE_H_

#include "ScenarioFeatureType.h"
#include "ScenarioFeatureDistributionType.h"
#include <string>
#include <vector>
#include <tinyxml.h>

#include "utils/NumberSampler.h"


class ScenarioFeature {

	ScenarioFeatureType m_featureType;
	ScenarioFeatureDistributionType m_distType;
	float m_dist_param_1;
	float m_dist_param_2;

	bool was_rolled_flag;
	float rolled_value;

public:
	ScenarioFeature();
	ScenarioFeature(std::string featureType);
	ScenarioFeature(ScenarioFeature * source_ScenarioFeature);
	ScenarioFeature(TiXmlNode* xml_ScenarioFeature);
	virtual ~ScenarioFeature();

	void roll();

	ScenarioFeatureType get_featureType();
	void set_featureType(ScenarioFeatureType type);

	ScenarioFeatureDistributionType get_distType();
	void set_distributionType(ScenarioFeatureDistributionType type);

	float get_dist_param_1();
	void set_dist_param_1(float val);

	float get_dist_param_2();
	void set_dist_param_2(float val);

	inline bool get_WasRolledFlag()
		{ return(rolled_value); }

	inline float get_RolledValue()
		{ if (was_rolled_flag) return(rolled_value); else return(0); }

	inline void set_RolledValue(float val)
		{ if (was_rolled_flag) { std::cout << "\033[1;31m !!! Replacing previous value !!! \033[0m" << std::endl; }
		  rolled_value = val;  was_rolled_flag = true;
		}

	int parseScenarioFeatureFromXML(TiXmlNode* xmlFeature);
	TiXmlElement *toXMLElement();
	TiXmlElement *toSFV_XMLElement();
};

#endif /* SCENARIOFEATURE_H_ */




