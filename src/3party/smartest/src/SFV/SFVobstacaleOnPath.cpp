/*
 * SFVobstacaleOnPath.cpp
 *
 *  Created on: Aug 5, 2014
 *      Author: userws3
 */



#include <iostream>
#include <vector>
#include "SFV/SFV.h"
#include "SFV/SFVobstacleOnPath.h"
#include "SFV/SFVplatformPose.h"
#include "SFV/SFVpath.h"
#include "SFV/SFVterraine.h"

#include "Generators/Gazebo/TerrainAnalyzer.h"
#include "Resource/ResourceHandler.h"

#include "SFDP/ScenarioFeatureType.h"
#include "SFDP/ScenarioFeature.h"
#include "SFDP/ScenarioFeatureGroup.h"


void SFVObstacleOnPath::initFeaturesMap()
{
	std::map<ScenarioFeatureType ,ScenarioFeature** > * my_features_map = get_FeaturesMap();

    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::obstacle_on_path_i_type, & my_obstacle_type ) );
    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::obstacle_on_path_i_scaling_factor, & my_scaling_factor));

    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::obstacle_on_path_i_location_along_the_path, & my_location_along_the_path));
    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::obstacle_on_path_i_location_perpendicular_to_the_path, & my_location_perpendicular_to_the_path));
    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::obstacle_on_path_i_location_on_the_Z_axis, & my_location_on_the_Z_axis));

    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::obstacle_on_path_i_location_Roll,  & my_location_Roll));
    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::obstacle_on_path_i_location_Pitch, & my_location_Pitch));
    my_features_map->insert(std::pair<ScenarioFeatureType,ScenarioFeature**>(ScenarioFeatureType::obstacle_on_path_i_location_Yaw, & my_location_Yaw));
}



SFVObstacleOnPath::SFVObstacleOnPath(ScenarioFeatureGroup * scenfeaturesGroup, SFV * parent_SFV): sfvSubGroup(ScenarioFeatureGroupType::obstacle_on_path, parent_SFV)
{
	set_Name(scenfeaturesGroup->get_name());
	initFeaturesMap();
	initSubGroupFeatures(scenfeaturesGroup->get_features());
	set_WasRolledFlag(false);

	Implicit_Obstacle_xyz = new std::map<char,float>;
}


SFVObstacleOnPath::SFVObstacleOnPath(SFVObstacleOnPath * template_subGroup):  sfvSubGroup(template_subGroup->get_Type(), template_subGroup->get_ParentSFV())
{
	initFeaturesMap();
	cloneSubGroupFeatures(template_subGroup);
	set_WasRolledFlag(false);

	Implicit_Obstacle_xyz = new std::map<char,float>;
}

SFVObstacleOnPath::SFVObstacleOnPath(TiXmlNode * xml_subGroup, SFV * parent_SFV): sfvSubGroup(ScenarioFeatureGroupType::obstacle_on_path , parent_SFV)
{
	initFeaturesMap();
	setSubGroupFeaturesFromXmlElement(xml_subGroup);
	set_WasRolledFlag(true);

	Implicit_Obstacle_xyz = new std::map<char,float>;
}


bool SFVObstacleOnPath::roll()
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




TiXmlElement * SFVObstacleOnPath::ToXmlElement(int id)
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



std::map<char,float> * SFVObstacleOnPath::get_Obstacle_xyz()
{

	SFV * sfv = get_ParentSFV();
	if(! sfv->get_WasRolledFlag())
	{
		std::cout << " can't calculate WP_xyz as the SFV wasn't fully rolled " << std::endl;
		return 0;
	}

	if(! Implicit_Obstacle_xyz->empty())
	{
		return(Implicit_Obstacle_xyz);
	}

	SFVplatformPose *sfv_platPose = (SFVplatformPose*)(sfv->get_SubGroupByFeatureGroupType(ScenarioFeatureGroupType::platform_pose));
	SFVpath *sfv_path = (SFVpath*)(sfv->get_SubGroupByFeatureGroupType(ScenarioFeatureGroupType::Path));

	double plat_x=sfv_platPose->get_PlatInit_xy()->at('x');
	double plat_y=sfv_platPose->get_PlatInit_xy()->at('y');
	double plat_init_azi = sfv_platPose->get_InitPlatformPoseAzimut()->get_RolledValue();


	double path_length = sfv_path->get_PathLength();
	double alo = this->get_LocationAlongThePath()->get_RolledValue();
	double per = this->get_LocationmyPerpendicularToPath()->get_RolledValue();
	double alo_path_pos = alo * path_length;
	//std::cout <<  "alo = " << alo <<  "  per = " << per << std::endl;

	double alon_path_dis = 0, azi = plat_init_azi, dis = 0, x=plat_x, y=plat_y, next_x=plat_x, next_y=plat_y;
	for (SFVwp *wp_it : *(sfv_path->get_PathWPs()) )
		{
		dis = wp_it->get_RalativeDistance()->get_RolledValue();
		next_x = wp_it->get_WPxy()->at('x');
		next_y = wp_it->get_WPxy()->at('y');

		if ( (alon_path_dis + dis) < alo_path_pos  )
			{
				alon_path_dis = alon_path_dis + dis;
				x = next_x;
				y = next_y;
				//std::cout <<  "wp_x = " << x <<  "  wp_y = " << y << std::endl;
			}
		else
			{
			break;
			}
		}

		double rem_alo = ((alo - alon_path_dis/path_length)*path_length)/dis;
		double obs_x = x + rem_alo * (next_x - x) - per*(next_y - y)/dis;
		double obs_y = y + rem_alo * (next_y - y) + per*(next_x - x)/dis;

		// std::cout <<  "obs_x = " << obs_x <<  "  obs_y = " << obs_y << std::endl;

		SFVterraine *sfv_terraine = (SFVterraine*)(sfv->get_SubGroupByFeatureGroupType(ScenarioFeatureGroupType::map));
		std::string terrain=ResourceHandler::getInstance(sfv->get_ResourceFile()).getTerrainById(sfv_terraine->get_TopographicMapIndex()->get_RolledValue());
		std::string path = ResourceHandler::getInstance(sfv->get_ResourceFile()).getWorldModelsFolderURL();


		TerrainAnalyzer m_terrainAnalyzer;
		m_terrainAnalyzer.loadFile(path+"/"+terrain);
		float terrain_z;
		m_terrainAnalyzer.getZCoord(obs_x,obs_y,terrain_z);
		double obs_z = terrain_z + this->get_LocationOnTheZaxis()->get_RolledValue();

		Implicit_Obstacle_xyz->insert(std::pair<char,float>('x',obs_x) );
		Implicit_Obstacle_xyz->insert(std::pair<char,float>('y',obs_y) );
		Implicit_Obstacle_xyz->insert(std::pair<char,float>('z',obs_z) );
		return(Implicit_Obstacle_xyz);
}


SFVObstacleOnPath::~SFVObstacleOnPath()
{

}



