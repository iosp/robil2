/*
 * SFVobjScattering.h
 *
 *  Created on: Jul 31, 2014
 *      Author: userws3
 */

#ifndef SFVOBJSCATTERING_H_
#define SFVOBJSCATTERING_H_


#include <vector>

#include "SFDP/ScenarioFeature.h"
#include "SFV/SFVObject.h"
#include "SFV/SFV.h"


class SFVobjScattering : public sfvSubGroup  {
private :
	ScenarioFeature * my_num_of_objects;

	SFVObject * my_object_template;
	std::vector <SFVObject *> * my_Objects;

public :
	SFVobjScattering(ScenarioFeatureGroup * scenfeaturesGroup,  SFV * parent_SFV);
	SFVobjScattering(SFVobjScattering * template_SFVobjScattering);
	SFVobjScattering(TiXmlNode * xml_subGroup, SFV * parent_SFV);

	void initFeaturesMap();

	bool roll();
	TiXmlElement * ToXmlElement(int id);

	~SFVobjScattering();

	inline ScenarioFeature * get_NumberOfObjects()
		{ return(my_num_of_objects); }

	inline std::vector<SFVObject *> * get_Objects()
		{ if (! my_Objects->empty() ) { return my_Objects; }
		  else { std::cout << "\033[1;31m Objects vector is empty \033[0m" << std::endl; return(0); } }

	inline SFVObject * get_ObjectTemplate()
		{ return(my_object_template); }

};


#endif /* SFVOBJSCATTERING_H_ */
