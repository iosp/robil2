/*
 * SFVObject.cpp
 *
 *  Created on: Jul 31, 2014
 *      Author: userws3
 */



#include <iostream>
#include <vector>
#include <map>

#include "SFV/SFVObject.h"
#include "SFDP/ScenarioFeatureType.h"
#include "SFDP/ScenarioFeature.h"
#include "SFDP/ScenarioFeatureGroup.h"

#include "SFV/sfvSubGroup.h"
#include "SFV/SFVterraine.h"

#include "Generators/Gazebo/TerrainAnalyzer.h"
#include "Resource/ResourceHandler.h"


void SFVObject::initFeaturesMap()
{
	std::map<ScenarioFeatureType ,ScenarioFeature** > * my_features_map = get_FeaturesMap();

    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::object_i_type, & my_object_type ) );
    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::object_i_scaling_factor, & my_scaling_factor));

    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::object_i_location_on_the_X_axis, & my_location_on_the_X_axis));
    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::object_i_location_on_the_Y_axis, & my_location_on_the_Y_axis));
    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::object_i_location_on_the_Z_axis, & my_location_on_the_Z_axis));

    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::object_i_location_Roll,  & my_location_Roll));
    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::object_i_location_Pitch, & my_location_Pitch));
    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::object_i_location_Yaw, & my_location_Yaw));
}


SFVObject::SFVObject(ScenarioFeatureGroup * scenfeaturesGroup, SFV * parent_SFV): sfvSubGroup(ScenarioFeatureGroupType::object, parent_SFV)
{
	set_Name(scenfeaturesGroup->get_name());
	initFeaturesMap();
	initSubGroupFeatures(scenfeaturesGroup->get_features());
	set_WasRolledFlag(false);

	Implicit_Object_xyz = new std::map<char,float>;
}


SFVObject::SFVObject(SFVObject * template_subGroup):  sfvSubGroup(template_subGroup->get_Type(), template_subGroup->get_ParentSFV())
{
	initFeaturesMap();
	cloneSubGroupFeatures(template_subGroup);
	set_WasRolledFlag(false);

	Implicit_Object_xyz = new std::map<char,float>;
}


SFVObject::SFVObject(TiXmlNode * xml_subGroup, SFV * parent_SFV): sfvSubGroup(ScenarioFeatureGroupType::object , parent_SFV)
{
	initFeaturesMap();
	setSubGroupFeaturesFromXmlElement(xml_subGroup);
	set_WasRolledFlag(true);

	Implicit_Object_xyz = new std::map<char,float>;
}


bool SFVObject::roll()
{
	if (get_WasRolledFlag())
	{
		std::cout << "\033[1;31m I already was rolled (I am Object) \033[0m"<< std::endl;
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
			return(true);
			std::cout << "\033[1;32m Succeed to roll " << this->get_Type() << " attempt = " << roll_attemp << " / " << roll_attemps_limit << "\033[0m"<< std::endl;
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

TiXmlElement * SFVObject::ToXmlElement(int id)
{
	if (! get_WasRolledFlag())
	{
		std::cout << "\033[1;31m can not make XML element for SFVObject because it wasn't rolled yet \033[0m"<< std::endl;
		return(0);
	}
	else
	{
		return(SubGroupfeaturesToXmlElement(id));
	}
}



std::map<char,float> * SFVObject::get_Object_xyz()
{
	SFV * sfv = get_ParentSFV();
	if(! sfv)
	{
		std::cout << " can't calculate Object_xyz as the SFV wasn't fully rolled " << std::endl;
		return 0;
	}

	if(! Implicit_Object_xyz->empty())
	{
		return(Implicit_Object_xyz);
	}

		SFVterraine *sfv_terraine = (SFVterraine*)(sfv->get_SubGroupByFeatureGroupType(ScenarioFeatureGroupType::map));
		std::string terrain=ResourceHandler::getInstance(sfv->get_ResourceFile()).getTerrainById(sfv_terraine->get_TopographicMapIndex()->get_RolledValue());
		std::string path = ResourceHandler::getInstance(sfv->get_ResourceFile()).getWorldModelsFolderURL();
		TerrainAnalyzer m_terrainAnalyzer;
		m_terrainAnalyzer.loadFile(path+"/"+terrain);

		float obj_x, obj_y, map_z;
		m_terrainAnalyzer.getXYZCoord(this->get_LocationOnTheXaxis()->get_RolledValue(),this->get_LocationOnTheYaxis()->get_RolledValue(), obj_x, obj_y, map_z);
		float obj_z = map_z + this->get_LocationOnTheZaxis()->get_RolledValue();

		Implicit_Object_xyz->clear();
		Implicit_Object_xyz->insert(std::pair<char,float>('x',obj_x) );
		Implicit_Object_xyz->insert(std::pair<char,float>('y',obj_y) );
		Implicit_Object_xyz->insert(std::pair<char,float>('z',obj_z) );
		return(Implicit_Object_xyz);
}





SFVObject::~SFVObject()
{

}




