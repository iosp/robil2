/*
 * SFVwp.h
 *
 *  Created on: Jul 30, 2014
 *      Author: userws3
 */

#ifndef SFVWP_H_
#define SFVWP_H_


#include "SFV/sfvSubGroup.h"
#include "SFDP/ScenarioFeature.h"
#include "SFV/SFV.h"


class SFVwp : public sfvSubGroup  {
private :
	ScenarioFeature * my_relative_angle;
	ScenarioFeature * my_relative_distance;

	std::map<char,float> * Implicit_wp_xy;

public :
	SFVwp(ScenarioFeatureGroup * scenfeaturesGroup, SFV * parent_SFV);
	SFVwp(SFVwp * template_SFVwp);
	SFVwp(TiXmlNode * xml_subGroup, SFV * parent_SFV);

	void initFeaturesMap();

	bool roll();
	std::map<char,float> * get_WPxy();
	TiXmlElement * ToXmlElement(int id);

	~SFVwp();


	inline ScenarioFeature * get_RalativeAngel()
		{ return(my_relative_angle); }

	inline ScenarioFeature * get_RalativeDistance()
		{ return(my_relative_distance); }

};


#endif /* SFVWP_H_ */
