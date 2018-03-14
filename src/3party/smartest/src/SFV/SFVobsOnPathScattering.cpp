/*
 * SFVobsOnPathScattering.cpp
 *
 *  Created on: Aug 5, 2014
 *      Author: userws3
 */

#include <iostream>
#include <utils/TinyXmlDef.h>


#include "SFV/SFVobsOnPathScattering.h"
#include "SFV/SFVobstacleOnPath.h"

#include "SFDP/ScenarioFeatureGroup.h"
#include "SFDP/ScenarioFeature.h"


void SFVobsOnPathScattering::initFeaturesMap()
{
	std::map<ScenarioFeatureType ,ScenarioFeature** > * my_features_map = get_FeaturesMap();

    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::number_of_obstacles_on_path, & my_num_of_obsOnpath ) );
}

SFVobsOnPathScattering::SFVobsOnPathScattering(ScenarioFeatureGroup * scenfeaturesGroup,  SFV * parent_SFV): sfvSubGroup(ScenarioFeatureGroupType::obstacles_on_path, parent_SFV)
{
	initFeaturesMap();
	initSubGroupFeatures(scenfeaturesGroup->get_features());

	set_Name(scenfeaturesGroup->get_name());
	my_obsOnPath_template = new SFVObstacleOnPath(scenfeaturesGroup, parent_SFV);
	my_ObstaclesOnpath = new std::vector<SFVObstacleOnPath *>;

	set_WasRolledFlag(false);
}

SFVobsOnPathScattering::SFVobsOnPathScattering(SFVobsOnPathScattering * template_subGroup): sfvSubGroup(template_subGroup->get_Type(),template_subGroup->get_ParentSFV())
{
	initFeaturesMap();
	cloneSubGroupFeatures(template_subGroup);

	my_obsOnPath_template = new SFVObstacleOnPath(template_subGroup->get_ObstaceOnPathTemplate());
	my_ObstaclesOnpath = new std::vector<SFVObstacleOnPath *>;

	set_WasRolledFlag(false);
}

SFVobsOnPathScattering::SFVobsOnPathScattering(TiXmlNode * xml_subGroup, SFV * parent_SFV): sfvSubGroup(ScenarioFeatureGroupType::obstacles_on_path , parent_SFV)
{
	initFeaturesMap();
	my_num_of_obsOnpath = new ScenarioFeature("number_of_obstacles_on_path");
	my_ObstaclesOnpath = new std::vector<SFVObstacleOnPath *>;

	TiXmlNode* XmlSubGroup_it = 0 ;
	int num_of_obs = 0;
	for ( XmlSubGroup_it = xml_subGroup->FirstChild(); XmlSubGroup_it != 0; XmlSubGroup_it = XmlSubGroup_it->NextSibling())
		{
		if (XmlSubGroup_it->Type()==XML_ELEMENT)
			{
			my_ObstaclesOnpath->push_back( new SFVObstacleOnPath(XmlSubGroup_it,parent_SFV));
			num_of_obs++;
			}
		}
	my_num_of_obsOnpath->set_RolledValue(num_of_obs);
	set_WasRolledFlag(true);
}

bool SFVobsOnPathScattering::roll()
{
	if (get_WasRolledFlag())
	{
		std::cout << "\033[1;31m I already was rolled (I am Obstacles On Path Scattering) \033[0m"<< std::endl;
		return(false);
	}
	else
	{
		int roll_attemps_limit = 3;
		int roll_attemp=1;
		bool roll_fail_flag=false;
		my_ObstaclesOnpath = new std::vector<SFVObstacleOnPath *>;

		while (roll_attemp <= roll_attemps_limit)
		{
			my_num_of_obsOnpath->roll();
			SFVObstacleOnPath * obs_i;
			for (int i=1 ; i<=my_num_of_obsOnpath->get_RolledValue(); i++)
				{
				obs_i = new SFVObstacleOnPath(my_obsOnPath_template);
				my_ObstaclesOnpath->push_back(obs_i);
				if (! obs_i->roll())
					{
					my_ObstaclesOnpath->pop_back();
					roll_fail_flag=true;
					break;
					}
				}

		if (roll_fail_flag)
			{
			my_num_of_obsOnpath = new ScenarioFeature(this->get_NumberOfObstaclesOnPath());
			my_ObstaclesOnpath = new std::vector<SFVObstacleOnPath *>;
			std::cout << "\033[1;35m fail to roll " << this->get_Type() << " attempt = " << roll_attemp << " / " << roll_attemps_limit << "\033[0m"<< std::endl;
			}
		else
			{
			set_WasRolledFlag(true);
			std::cout << "\033[1;32m Succeed to roll " << this->get_Type() << " attempt = " << roll_attemp << " / " << roll_attemps_limit << "\033[0m"<< std::endl;
			return(true);
			}
		roll_attemp++;
		}
	return(false);
	}
}


TiXmlElement * SFVobsOnPathScattering::ToXmlElement(int id)
{
	if (! get_WasRolledFlag())
	{
		std::cout << "\033[1;31m can not make XML element for SFVobsOnPathScattering because it wasn't rolled yet \033[0m"<< std::endl;
		return(0);
	}
	else
	{
		TiXmlElement * xml_sub_group = new TiXmlElement(get_Type().str());
		xml_sub_group->SetAttribute("ID",std::to_string(id));

		int id=1;
		if ( get_NumberOfObstaclesOnPath()->get_RolledValue() > 0 )
		{
			for (SFVObstacleOnPath * obs_it : * get_ObstaclesOnPath())
			{
				xml_sub_group->LinkEndChild(obs_it->ToXmlElement(id));
				id++;
			}
		}

		return(xml_sub_group);
	}
}

SFVobsOnPathScattering::~SFVobsOnPathScattering()
{

}



