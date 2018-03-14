/*
 * SFVfriction_link.cpp
 *
 *  Created on: Aug 5, 2014
 *      Author: userws3
 */



#include <iostream>
#include "SFV/SFVfriction_link.h"
#include "SFDP/ScenarioFeatureGroup.h"
#include "SFDP/ScenarioFeatureType.h"

#include "SFV/SFV.h"

void SFVfriction_link::initFeaturesMap()
{
	std::map<ScenarioFeatureType ,ScenarioFeature** > * my_features_map = get_FeaturesMap();

    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::friction_link_friction_deviation, & friction_deviation ) );
}

SFVfriction_link::SFVfriction_link(ScenarioFeatureGroup * scenfeaturesGroup, SFV * parent_SFV): sfvSubGroup(ScenarioFeatureGroupType::friction_link_i, parent_SFV)
{
	set_Name(scenfeaturesGroup->get_name());
	initFeaturesMap();
	initSubGroupFeatures(scenfeaturesGroup->get_features());
	set_WasRolledFlag(false);
}


SFVfriction_link::SFVfriction_link(SFVfriction_link * template_subGroup): sfvSubGroup(template_subGroup->get_Type() ,template_subGroup->get_ParentSFV())
{
	set_Name(template_subGroup->get_Name());
	initFeaturesMap();
	cloneSubGroupFeatures(template_subGroup);
	set_WasRolledFlag(false);
}

SFVfriction_link::SFVfriction_link(TiXmlNode * xml_subGroup, SFV * parent_SFV): sfvSubGroup(ScenarioFeatureGroupType::friction_link_i , parent_SFV)
{
	initFeaturesMap();
	setSubGroupFeaturesFromXmlElement(xml_subGroup);
	set_WasRolledFlag(true);
}

bool SFVfriction_link::roll()
{
	int roll_attemps_limit = 3;

	if (get_WasRolledFlag())
	{
		std::cout << "\033[1;31m I already was rolled (I am Friction Link) \033[0m"<< std::endl;
		return(false);
	}
	else
	{
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
			std::cout << "\033[1;35m fail to roll " << this->get_Type() << " attempt = " << roll_attemp << " / " << roll_attemps_limit << "\033[0m"<< std::endl;
			}
		roll_attemp++;
		}
	return(false);
	}
}

TiXmlElement * SFVfriction_link::ToXmlElement(int id)
{
	if (! get_WasRolledFlag())
	{
		std::cout << "\033[1;31m can not make XML element for SFVfriction_link because it wasn't rolled yet \033[0m"<< std::endl;
		return(0);
	}
	else
	{
		return(SubGroupfeaturesToXmlElement(id));
	}
}



SFVfriction_link::~SFVfriction_link()
{

}



