/*
 * SFVmass_link.cpp
 *
 *  Created on: Jul 30, 2014
 *      Author: userws3
 */


#include <iostream>
#include "SFV/SFVmass_link.h"
#include "SFDP/ScenarioFeatureGroup.h"
#include "SFDP/ScenarioFeatureType.h"

#include "SFV/SFV.h"


void SFVmass_link::initFeaturesMap()
{
	std::map<ScenarioFeatureType ,ScenarioFeature** > * my_features_map = get_FeaturesMap();

    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::mass_link_i_mass_deviation, & mass_deviation ) );

    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::mass_link_i_inertia_deviation_Ixx_component, & inertia_deviation_Ixx_component ) );
    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::mass_link_i_inertia_deviation_Iyy_component, & inertia_deviation_Iyy_component ) );
    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::mass_link_i_inertia_deviation_Izz_component, & inertia_deviation_Izz_component ) );

    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::mass_link_i_location_deviation_X, & location_deviation_X ) );
    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::mass_link_i_location_deviation_Y, & location_deviation_Y ) );
    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::mass_link_i_location_deviation_Z, & location_deviation_Z ) );

    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::mass_link_i_location_deviation_Roll, & location_deviation_Roll ) );
    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::mass_link_i_location_deviation_Pitch, & location_deviation_Pitch ) );
    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::mass_link_i_location_deviation_Yaw, & location_deviation_Yaw ) );
}


SFVmass_link::SFVmass_link(ScenarioFeatureGroup * scenfeaturesGroup, SFV * parent_SFV): sfvSubGroup(ScenarioFeatureGroupType::mass_link_i, parent_SFV)
{
	set_Name(scenfeaturesGroup->get_name());
	initFeaturesMap();
	initSubGroupFeatures(scenfeaturesGroup->get_features());
	set_WasRolledFlag(false);
}


SFVmass_link::SFVmass_link(SFVmass_link * template_subGroup): sfvSubGroup(template_subGroup->get_Type() ,template_subGroup->get_ParentSFV())
{
	set_Name(template_subGroup->get_Name());
	initFeaturesMap();
	cloneSubGroupFeatures(template_subGroup);
	set_WasRolledFlag(false);
}

SFVmass_link::SFVmass_link(TiXmlNode * xml_subGroup, SFV * parent_SFV): sfvSubGroup(ScenarioFeatureGroupType::mass_link_i , parent_SFV)
{
	initFeaturesMap();
	setSubGroupFeaturesFromXmlElement(xml_subGroup);
	set_WasRolledFlag(true);
}

bool SFVmass_link::roll()
{
	int roll_attemps_limit = 3;

	if (get_WasRolledFlag())
	{
		std::cout << "\033[1;31m I already was rolled (I am Mass Link) \033[0m"<< std::endl;
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

TiXmlElement * SFVmass_link::ToXmlElement(int id)
{
	if (!get_WasRolledFlag())
	{
		std::cout << "\033[1;31m can not make XML element for SFVmass_link because it wasn't rolled yet \033[0m"<< std::endl;
		return(0);
	}
	else
	{
		return(SubGroupfeaturesToXmlElement(id));
	}
}



SFVmass_link::~SFVmass_link()
{

}



