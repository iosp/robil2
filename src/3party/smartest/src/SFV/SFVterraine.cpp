/*
 * SFVterraine.cpp
 *
 *  Created on: Jul 31, 2014
 *      Author: userws3
 */



#include <iostream>
#include "SFV/SFVterraine.h"

#include "SFDP/ScenarioFeatureGroup.h"
#include "SFDP/ScenarioFeatureType.h"


void SFVterraine::initFeaturesMap()
{
	std::map<ScenarioFeatureType ,ScenarioFeature** > * my_features_map = get_FeaturesMap();

    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::topographic_map, & topographic_map_index ) );
}

SFVterraine::SFVterraine(ScenarioFeatureGroup * scenfeaturesGroup, SFV * parent_SFV): sfvSubGroup(ScenarioFeatureGroupType::map , parent_SFV)
{
	set_Name(scenfeaturesGroup->get_name());
	initFeaturesMap();
	initSubGroupFeatures(scenfeaturesGroup->get_features());
	set_WasRolledFlag(false);
}


SFVterraine::SFVterraine(SFVterraine * template_subGroup): sfvSubGroup(template_subGroup->get_Type(), template_subGroup->get_ParentSFV())
{
	initFeaturesMap();
	cloneSubGroupFeatures(template_subGroup);
	set_WasRolledFlag(false);
}


SFVterraine::SFVterraine(TiXmlNode * xml_subGroup, SFV * parent_SFV): sfvSubGroup(ScenarioFeatureGroupType::map , parent_SFV)
{
	initFeaturesMap();
	setSubGroupFeaturesFromXmlElement(xml_subGroup);
	set_WasRolledFlag(true);
}


bool SFVterraine::roll()
{
	if (get_WasRolledFlag())
	{
		std::cout << "\033[1;31m I already was rolled (I am Typographic Map) \033[0m"<< std::endl;
		return(false);
	}
	else
	{
		int roll_attemps_limit = 3;
		int roll_attemp=1;
		while (roll_attemp <= roll_attemps_limit)
		{
			rollSubGroupfeatures();

		if (get_ParentSFV()->rules_check())
			{
			set_WasRolledFlag(true);
			std::cout << "\033[1;32m Succeed to roll " << this->get_Type() << " attempt = " << roll_attemp << " / " << roll_attemps_limit << "\033[0m"<< std::endl;
			return(true);
			}
		else
			{
			resetSubGroupfeatures();
			std::cout << "\033[1;35m Fail to roll " << this->get_Type() << " attempt = " << roll_attemp << " / " << roll_attemps_limit << "\033[0m"<< std::endl;
			}
		roll_attemp++;
		}
	return(false);
	}
}

TiXmlElement * SFVterraine::ToXmlElement(int id)
{
	if (! get_WasRolledFlag())
	{
		std::cout << "\033[1;31m can not make XML element for SFVterraine because it wasn't rolled yet \033[0m"<< std::endl;
		return(0);
	}
	else
	{
		return(SubGroupfeaturesToXmlElement(id));
	}
}

SFVterraine::~SFVterraine()
{

}



