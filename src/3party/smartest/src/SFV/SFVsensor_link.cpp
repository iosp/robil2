/*
 * SFVsensor_link.cpp
 *
 *  Created on: Aug 5, 2014
 *      Author: userws3
 */


#include <iostream>
#include "SFV/SFVsensor_link.h"
#include "SFDP/ScenarioFeatureGroup.h"
#include "SFDP/ScenarioFeatureType.h"

#include "SFV/SFV.h"


void SFVsensor_link::initFeaturesMap()
{
	std::map<ScenarioFeatureType ,ScenarioFeature** > * my_features_map = get_FeaturesMap();

    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::sensor_link_i_location_deviation_X, & location_deviationX ) );
    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::sensor_link_i_location_deviation_Y, & location_deviationY ) );
    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::sensor_link_i_location_deviation_Z, & location_deviationZ ) );

    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::sensor_link_i_location_deviation_Roll, & location_deviationRoll ) );
    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::sensor_link_i_location_deviation_Pitch, & location_deviationPitch ) );
    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::sensor_link_i_location_deviation_Yaw, & location_deviationYaw ) );
}

SFVsensor_link::SFVsensor_link(ScenarioFeatureGroup * scenfeaturesGroup, SFV * parent_SFV): sfvSubGroup(ScenarioFeatureGroupType::sensor_link_i, parent_SFV)
{
	set_Name(scenfeaturesGroup->get_name());
	initFeaturesMap();
	initSubGroupFeatures(scenfeaturesGroup->get_features());
	set_WasRolledFlag(false);
}


SFVsensor_link::SFVsensor_link(SFVsensor_link * template_subGroup): sfvSubGroup(template_subGroup->get_Type() ,template_subGroup->get_ParentSFV())
{
	set_Name(template_subGroup->get_Name());
	initFeaturesMap();
	cloneSubGroupFeatures(template_subGroup);
	set_WasRolledFlag(false);
}

SFVsensor_link::SFVsensor_link(TiXmlNode * xml_subGroup, SFV * parent_SFV): sfvSubGroup(ScenarioFeatureGroupType::sensor_link_i , parent_SFV)
{
	initFeaturesMap();
	setSubGroupFeaturesFromXmlElement(xml_subGroup);
	set_WasRolledFlag(true);
}


bool SFVsensor_link::roll()
{
	int roll_attemps_limit = 3;

	if (get_WasRolledFlag())
	{
		std::cout << "\033[1;31m I already was rolled (I am Sensor Link) \033[0m"<< std::endl;
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

TiXmlElement * SFVsensor_link::ToXmlElement(int id)
{
	if (! get_WasRolledFlag())
	{
		std::cout << "\033[1;31m can not make XML element for SFVsensor_link because it wasn't rolled yet \033[0m"<< std::endl;
		return(0);
	}
	else
	{
		return(SubGroupfeaturesToXmlElement(id));
	}
}



SFVsensor_link::~SFVsensor_link()
{

}




