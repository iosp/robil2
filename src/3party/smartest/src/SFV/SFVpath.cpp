/*
 * SFVpath.cpp
 *
 *  Created on: Jul 30, 2014
 *      Author: userws3
 */

#include <iostream>
#include <tinyxml.h>
#include <math.h>

#include "SFV/SFVpath.h"
#include "SFV/SFVwp.h"
#include "SFDP/ScenarioFeatureGroup.h"
#include "SFDP/ScenarioFeature.h"


void SFVpath::initFeaturesMap()
{
	std::map<ScenarioFeatureType ,ScenarioFeature** > * my_features_map = get_FeaturesMap();

    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::number_of_way_points, & my_number_of_wp ) );
}


SFVpath::SFVpath(ScenarioFeatureGroup * scenfeaturesGroup,  SFV * parent_SFV): sfvSubGroup(ScenarioFeatureGroupType::Path, parent_SFV)
{
	initFeaturesMap();
	initSubGroupFeatures(scenfeaturesGroup->get_features());

	set_Name(scenfeaturesGroup->get_name());
	my_wp_template = new SFVwp(scenfeaturesGroup, parent_SFV);
	my_PathWPs = new std::vector<SFVwp *>;

	set_WasRolledFlag(false);
}

SFVpath::SFVpath(SFVpath * template_subGroup): sfvSubGroup(template_subGroup->get_Type(),template_subGroup->get_ParentSFV())
{
	initFeaturesMap();
	cloneSubGroupFeatures(template_subGroup);

	my_wp_template = new SFVwp(template_subGroup->get_WpTemplate());
	my_PathWPs = new std::vector<SFVwp *>;

	set_WasRolledFlag(false);
}

SFVpath::SFVpath(TiXmlNode * xml_subGroup, SFV * parent_SFV): sfvSubGroup(ScenarioFeatureGroupType::Path , parent_SFV)
{
	initFeaturesMap();
	my_number_of_wp = new ScenarioFeature("number_of_way_points");
	my_PathWPs = new std::vector<SFVwp *>;

	TiXmlNode* XmlSubGroup_it = 0 ;
	int num_of_wp = 0;
	for ( XmlSubGroup_it = xml_subGroup->FirstChild(); XmlSubGroup_it != 0; XmlSubGroup_it = XmlSubGroup_it->NextSibling())
		{
		if (XmlSubGroup_it->Type()==XML_ELEMENT)
			{
			my_PathWPs->push_back( new SFVwp(XmlSubGroup_it,parent_SFV));
			num_of_wp++;
			}
		}
	my_number_of_wp->set_RolledValue(num_of_wp);
	set_WasRolledFlag(true);
}

bool SFVpath::roll()
{
	if (get_WasRolledFlag())
	{
		std::cout << "\033[1;31m I already was rolled (I am Path) \033[0m"<< std::endl;
		return(false);
	}
	else
	{
		int roll_attemps_limit = 3;
		int roll_attemp=1;
		bool roll_fail_flag=false;
		my_PathWPs = new std::vector<SFVwp *>;

		while (roll_attemp <= roll_attemps_limit)
		{
			my_number_of_wp->roll();
			SFVwp * wp_i;
			for (int i=1 ; i<=my_number_of_wp->get_RolledValue(); i++)
				{
				wp_i = new SFVwp(my_wp_template);
				my_PathWPs->push_back(wp_i);
				if (! wp_i->roll())
					{
					my_PathWPs->pop_back();
					roll_fail_flag=true;
					break;
					}
				}

		if (roll_fail_flag)
			{
			my_number_of_wp = new ScenarioFeature(this->get_NumberOfWPs());
			my_PathWPs = new std::vector<SFVwp *>;
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


TiXmlElement * SFVpath::ToXmlElement(int id)
{
	if (! get_WasRolledFlag())
	{
		std::cout << "\033[1;31m can not make XML element for SFVpath because it wasn't rolled yet \033[0m"<< std::endl;
		return(0);
	}
	else
	{
		TiXmlElement * xml_sub_group = new TiXmlElement(get_Type().str());
		xml_sub_group->SetAttribute("ID",std::to_string(id));

		int id=1;
		for (SFVwp * wp_it : * get_PathWPs())
		{
			xml_sub_group->LinkEndChild(wp_it->ToXmlElement(id));
			id++;
		}

		return(xml_sub_group);
	}
}


float SFVpath::get_PathLength()
{
	SFV* sfv = get_ParentSFV();
	if(! sfv->get_WasRolledFlag())
	{
		std::cout << " can't calculate PathLength as the SFV wasn't fully rolled " << std::endl;
		return 0;
	}

	SFVpath *sfv_path = (SFVpath*)(sfv->get_SubGroupByFeatureGroupType(ScenarioFeatureGroupType::Path));

	float path_length = 0;

	for(SFVwp *wp_it : *(sfv_path->get_PathWPs()))
		{
		path_length = path_length + wp_it->get_RalativeDistance()->get_RolledValue();
		}

	return(path_length);
}

SFVpath::~SFVpath()
{

}

