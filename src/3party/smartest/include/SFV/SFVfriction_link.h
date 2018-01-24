/*
 * SFVfriction_link.h
 *
 *  Created on: Aug 5, 2014
 *      Author: userws3
 */

#ifndef SFVFRICTION_LINK_H_
#define SFVFRICTION_LINK_H_


#include <iostream>
#include <string>

#include "SFV/sfvSubGroup.h"
#include "SFDP/ScenarioFeature.h"
#include "SFV/SFV.h"

class SFVfriction_link : public sfvSubGroup {
private :

	std::string my_FrictionLinkName;

	ScenarioFeature * friction_deviation;

public :
	SFVfriction_link(ScenarioFeatureGroup * scenfeaturesGroup, SFV * parent_SFV);
	SFVfriction_link(SFVfriction_link * template_SFVfriction_link);
	SFVfriction_link(TiXmlNode * xml_subGroup, SFV * parent_SFV);

	void initFeaturesMap();

	bool roll();
	TiXmlElement * ToXmlElement(int id);

	~SFVfriction_link();

	inline ScenarioFeature * get_FrictionDeviation()
		{ return(friction_deviation); }

};


#endif /* SFVFRICTION_LINK_H_ */
