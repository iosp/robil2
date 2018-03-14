/*
 * SFVpath.h
 *
 *  Created on: Jul 30, 2014
 *      Author: userws3
 */

#ifndef SFVPATH_H_
#define SFVPATH_H_

#include <vector>

#include "SFDP/ScenarioFeature.h"
#include "SFV/SFVwp.h"
#include "SFV/SFV.h"
#include "utils/TinyXmlDef.h"



class SFVpath : public sfvSubGroup  {
private :
	ScenarioFeature * my_number_of_wp;
	SFVwp * my_wp_template;
	std::vector <SFVwp *> * my_PathWPs;

public :
	SFVpath(ScenarioFeatureGroup * scenfeaturesGroup,  SFV * parent_SFV);
	SFVpath(SFVpath * template_SFVpath);
	SFVpath(TiXmlNode * xml_subGroup, SFV * parent_SFV);

	void initFeaturesMap();

	bool roll();

	float get_PathLength();
	TiXmlElement * ToXmlElement(int id);

	~SFVpath();


	inline ScenarioFeature * get_NumberOfWPs()
		{ return(my_number_of_wp); }

	inline std::vector<SFVwp *> * get_PathWPs()
		{ return(my_PathWPs); }

	inline SFVwp * get_WpTemplate()
		{ return(my_wp_template); }


};


#endif /* SFVPATH_H_ */
