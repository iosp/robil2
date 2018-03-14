/*
 * SFVobjScattering.cpp
 *
 *  Created on: Jul 31, 2014
 *      Author: userws3
 */


#include <iostream>
#include "SFV/SFVobjScattering.h"
#include "SFV/SFVObject.h"
#include "SFDP/ScenarioFeatureGroup.h"
#include "SFDP/ScenarioFeature.h"
#include "utils/TinyXmlDef.h"



void SFVobjScattering::initFeaturesMap()
{
	std::map<ScenarioFeatureType ,ScenarioFeature** > * my_features_map = get_FeaturesMap();

    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::number_of_objects, & my_num_of_objects ) );
}


SFVobjScattering::SFVobjScattering(ScenarioFeatureGroup * scenfeaturesGroup,  SFV * parent_SFV): sfvSubGroup(ScenarioFeatureGroupType::objects, parent_SFV)
{
	initFeaturesMap();
	initSubGroupFeatures(scenfeaturesGroup->get_features());

	set_Name(scenfeaturesGroup->get_name());
	my_object_template = new SFVObject(scenfeaturesGroup, parent_SFV);
	my_Objects = new std::vector<SFVObject *>;

	set_WasRolledFlag(false);
}

SFVobjScattering::SFVobjScattering(SFVobjScattering * template_subGroup): sfvSubGroup(template_subGroup->get_Type(),template_subGroup->get_ParentSFV())
{
	initFeaturesMap();
	cloneSubGroupFeatures(template_subGroup);

	my_object_template = new SFVObject(template_subGroup->get_ObjectTemplate());
	my_Objects = new std::vector<SFVObject *>;

	set_WasRolledFlag(false);
}

SFVobjScattering::SFVobjScattering(TiXmlNode * xml_subGroup, SFV * parent_SFV): sfvSubGroup(ScenarioFeatureGroupType::objects , parent_SFV)
{
	initFeaturesMap();
	my_num_of_objects = new ScenarioFeature("number_of_objects");
	my_Objects = new std::vector<SFVObject *>;

	TiXmlNode* XmlSubGroup_it = 0 ;
	int num_of_objects = 0;
	for ( XmlSubGroup_it = xml_subGroup->FirstChild(); XmlSubGroup_it != 0; XmlSubGroup_it = XmlSubGroup_it->NextSibling())
		{
		if (XmlSubGroup_it->Type()==XML_ELEMENT)
			{
			my_Objects->push_back( new SFVObject(XmlSubGroup_it,parent_SFV) );
			num_of_objects++;
			}
		}
	my_num_of_objects->set_RolledValue(num_of_objects);
	set_WasRolledFlag(true);
}

bool SFVobjScattering::roll()
{
	if (get_WasRolledFlag())
	{
		std::cout << "\033[1;31m I already was rolled (I am Objects Scattering) \033[0m"<< std::endl;
		return(false);
	}
	else
	{
		int roll_attemps_limit = 3;
		int roll_attemp=1;
		bool roll_fail_flag=false;
		my_Objects = new std::vector<SFVObject *>;

		while (roll_attemp <= roll_attemps_limit)
		{
			my_num_of_objects->roll();
			SFVObject * obj_i;
			for (int i=1 ; i<=my_num_of_objects->get_RolledValue(); i++)
				{
				obj_i = new SFVObject(my_object_template);
				my_Objects->push_back(obj_i);
				if (! obj_i->roll())
					{
					my_Objects->pop_back();
					roll_fail_flag=true;
					break;
					}
				}

		if (roll_fail_flag)
			{
			my_num_of_objects = new ScenarioFeature(this->get_NumberOfObjects());
			my_Objects = new std::vector<SFVObject *>;
			std::cout << "\033[1;35m fail to roll " << this->get_Type() << " attempt = " << roll_attemp << " / " << roll_attemps_limit << "\033[0m"<< std::endl;
			}
		else
			{
			set_WasRolledFlag(true);
			std::cout << "\033[1;32m Succeed to roll " << this->get_Type() << " attempt = " << roll_attemp << " / " << roll_attemps_limit << "\033[0m"<< std::endl;

			std::cout << "my_num_of_objects->get_RolledValue() = " << my_num_of_objects->get_RolledValue() << std::endl;
			return(true);
			}
		roll_attemp++;
		}
	return(false);
	}
}


TiXmlElement * SFVobjScattering::ToXmlElement(int id)
{
	if (! get_WasRolledFlag())
	{
		std::cout << "\033[1;31m can not make XML element for SFVobjScattering because it wasn't rolled yet \033[0m"<< std::endl;
		return(0);
	}
	else
	{
		TiXmlElement * xml_sub_group = new TiXmlElement(get_Type().str());
		xml_sub_group->SetAttribute("ID",std::to_string(id));

		int id=1;
		if ( get_NumberOfObjects()->get_RolledValue() > 0 )
		{
			for (SFVObject * obj_it : * get_Objects())
			{
				xml_sub_group->LinkEndChild(obj_it->ToXmlElement(id));
				id++;
			}
		}

		return(xml_sub_group);
	}
}

SFVobjScattering::~SFVobjScattering()
{

}
