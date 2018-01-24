/*
 * SFVterraine.h
 *
 *  Created on: Jul 31, 2014
 *      Author: userws3
 */

#ifndef SFVTERRAINE_H_
#define SFVTERRAINE_H_


#include <iostream>
#include <string>

#include "SFV/sfvSubGroup.h"
#include "SFDP/ScenarioFeature.h"
#include "SFV/SFV.h"


class SFVterraine : public sfvSubGroup {
private :

	ScenarioFeature * topographic_map_index;

public :
//	SFVterraine(std::vector<ScenarioFeature *> * ScenarioFeatures_vec, SFV * parent_SFV);
	SFVterraine(ScenarioFeatureGroup * scenfeaturesGroup, SFV * parent_SFV);
	SFVterraine(SFVterraine * template_SFVterraine);
	SFVterraine(TiXmlNode * xml_subGroup, SFV * parent_SFV);

	void initFeaturesMap();

	bool roll();

	TiXmlElement * ToXmlElement(int id);

	~SFVterraine();

	inline ScenarioFeature * get_TopographicMapIndex()
		{ return(topographic_map_index); }

};




#endif /* SFVTERRAINE_H_ */
