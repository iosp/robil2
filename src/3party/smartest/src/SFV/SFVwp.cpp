/*
 * SFVwp.cpp
 *
 *  Created on: Jul 30, 2014
 *      Author: userws3
 */

#include <iostream>
#include <vector>
#include <map>
#include <tinyxml.h>

#include "SFV/sfvSubGroup.h"
#include "SFV/SFVwp.h"
#include "SFDP/ScenarioFeatureType.h"
#include "SFDP/ScenarioFeature.h"
#include "SFDP/ScenarioFeatureGroup.h"

#include "SFV/SFVpath.h"
#include "SFV/SFVplatformPose.h"
#include "SFV/SFVterraine.h"
#include "Resource/ResourceHandler.h"
#include "Generators/Gazebo/TerrainAnalyzer.h"



void SFVwp::initFeaturesMap()
{
	std::map<ScenarioFeatureType ,ScenarioFeature** > * my_features_map = get_FeaturesMap();

    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::wp_i_relative_angle, & my_relative_angle ) );
    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::wp_i_relative_distance, & my_relative_distance));

}


SFVwp::SFVwp(ScenarioFeatureGroup * scenfeaturesGroup, SFV * parent_SFV): sfvSubGroup(ScenarioFeatureGroupType::WayPoint, parent_SFV)
{
	set_Name(scenfeaturesGroup->get_name());
	initFeaturesMap();
	initSubGroupFeatures(scenfeaturesGroup->get_features());
	set_WasRolledFlag(false);

	Implicit_wp_xy = new std::map<char,float>;
}


SFVwp::SFVwp(SFVwp * template_subGroup): sfvSubGroup(template_subGroup->get_Type(),template_subGroup->get_ParentSFV())
{
	initFeaturesMap();
	cloneSubGroupFeatures(template_subGroup);
	set_WasRolledFlag(false);

	Implicit_wp_xy = new std::map<char,float>;
}

SFVwp::SFVwp(TiXmlNode * xml_subGroup, SFV * parent_SFV): sfvSubGroup(ScenarioFeatureGroupType::WayPoint , parent_SFV)
{
	initFeaturesMap();
	setSubGroupFeaturesFromXmlElement(xml_subGroup);
	set_WasRolledFlag(true);

	Implicit_wp_xy = new std::map<char,float>;
}


bool SFVwp::roll()
{
	if ( get_WasRolledFlag())
	{
		std::cout << "\033[1;31m I already was rolled (I am SFVwp) \033[0m"<< std::endl;
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
			std::cout << "\033[1;35m fail to roll " << this->get_Type() << " attempt = " << roll_attemp << " / " << roll_attemps_limit << "\033[0m"<< std::endl;
			}
		roll_attemp++;
		}
	return(false);
	}
}



TiXmlElement * SFVwp::ToXmlElement(int id)
{
	if (!  get_WasRolledFlag())
	{
		std::cout << "\033[1;31m can not make XML element for SFVwp because it wasn't rolled yet \033[0m"<< std::endl;
		return(0);
	}
	else
	{
		return(SubGroupfeaturesToXmlElement(id));
	}
}


std::map<char,float> * SFVwp::get_WPxy()
{
	SFV *sfv = get_ParentSFV();
	if(! sfv->get_WasRolledFlag())
	{
		std::cout << " can't calculate WP_xy as the SFV wasn't fully rolled " << std::endl;
		return 0;
	}

	if(! Implicit_wp_xy->empty())
	{
		return(Implicit_wp_xy);
	}

	SFVplatformPose *sfv_platPose = (SFVplatformPose*)(sfv->get_SubGroupByFeatureGroupType(ScenarioFeatureGroupType::platform_pose));
	SFVterraine *sfv_terraine = (SFVterraine*)(sfv->get_SubGroupByFeatureGroupType(ScenarioFeatureGroupType::map));
	SFVpath *sfv_Path = (SFVpath*)(sfv->get_SubGroupByFeatureGroupType(ScenarioFeatureGroupType::Path));


	//load terrain
	std::string terrain_name=ResourceHandler::getInstance(sfv->get_ResourceFile()).getTerrainById(sfv_terraine->get_TopographicMapIndex()->get_WasRolledFlag());
	std::string teraine_file_url = ResourceHandler::getInstance(sfv->get_ResourceFile()).getWorldModelsFolderURL();


	TerrainAnalyzer terrainA;
	terrainA.loadFile(teraine_file_url+"/"+terrain_name);

	//get platform initial position
	//float plat_init_x , plat_init_y, plat_init_z , plat_init_azi;
	//terrainA.getXYZCoord(sfv_platPose->get_InitPlatformPoseX()->get_RolledValue(),sfv_platPose->get_InitPlatformPoseY()->get_RolledValue(),plat_init_x, plat_init_y ,plat_init_z);
	float plat_init_x = sfv_platPose->get_PlatInit_xy()->at('x');
	float plat_init_y = sfv_platPose->get_PlatInit_xy()->at('y');
	float plat_init_azi = sfv_platPose->get_InitPlatformPoseAzimut()->get_RolledValue();

	float wp_x=plat_init_x, wp_y=plat_init_y,  wp_azi = plat_init_azi;

	for (SFVwp *wp_it : *(sfv_Path->get_PathWPs()) )
			{
			wp_azi = wp_azi + wp_it->get_RalativeAngel()->get_RolledValue();
			wp_x = wp_x + wp_it->get_RalativeDistance()->get_RolledValue()*cos(wp_azi);
			wp_y = wp_y + wp_it->get_RalativeDistance()->get_RolledValue()*sin(wp_azi);

			if(wp_it == this)
				{
				Implicit_wp_xy->clear();
				Implicit_wp_xy->insert(std::pair<char,float>('x',wp_x) );
				Implicit_wp_xy->insert(std::pair<char,float>('y',wp_y) );
				return(Implicit_wp_xy);
				}
			}

	std::cout << " the WP is not on Path " << std::endl;
	return(0);
}


SFVwp::~SFVwp()
{

}




