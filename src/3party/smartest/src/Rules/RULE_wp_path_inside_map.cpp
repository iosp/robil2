/*
 * RULE_WP_path_inside_map.cpp
 *
 *  Created on: Jun 23, 2014
 *      Author: userws3
 */


#include "Rules/RULE_wp_path_inside_map.h"

#include "Resource/ResourceHandler.h"
#include "Generators/Gazebo/TerrainAnalyzer.h"

#include "SFV/SFVpath.h"
#include "SFV/SFVplatformPose.h"
#include "SFV/SFVterraine.h"


Rule_wp_path_inside_map::Rule_wp_path_inside_map() {
	// TODO Auto-generated constructor stub
}

Rule_wp_path_inside_map::~Rule_wp_path_inside_map() {
	// TODO Auto-generated destructor stub
}


bool Rule_wp_path_inside_map::isRuleValid(SFV *sfv)
{
	std::cout << " !! checking Rule_wp_path_inside_map --- " ;


	SFVpath *sfv_Path = (SFVpath*)(sfv->get_SubGroupByFeatureGroupType(ScenarioFeatureGroupType::Path));
	if (sfv_Path->get_PathWPs()->empty())
		{
		std::cout << " path first wp hasn't been rolled yet, so the rule has no meaning for now - return true " << std::endl;
		return true;
		}
	else
		{
			SFVterraine *sfv_terraine = (SFVterraine*)(sfv->get_SubGroupByFeatureGroupType(ScenarioFeatureGroupType::map));
			if (! sfv_terraine->get_WasRolledFlag())
				{
				std::cout << " terrain hasn't been rolled yet, so the rule can't be evaluated - return false" << std::endl;
				return false;
				}

			SFVplatformPose *sfv_platPose = (SFVplatformPose*)(sfv->get_SubGroupByFeatureGroupType(ScenarioFeatureGroupType::platform_pose));
			if (! sfv_platPose->get_WasRolledFlag())
				{
				std::cout << " platform initial position hasn't been rolled yet, so the rule can't be evaluated - return false" << std::endl;
				return false;
				}

		//load terrain
		std::string terrain_name=ResourceHandler::getInstance(sfv->get_ResourceFile()).getTerrainById(sfv_terraine->get_TopographicMapIndex()->get_RolledValue());
		std::string teraine_file_url = ResourceHandler::getInstance(sfv->get_ResourceFile()).getWorldModelsFolderURL();
		TerrainAnalyzer* terrainA=new TerrainAnalyzer();
		terrainA->loadFile(teraine_file_url+"/"+terrain_name);

		//get platform initial position
		float plat_init_x , plat_init_y, plat_init_z , plat_init_azi;
		terrainA->getXYZCoord(sfv_platPose->get_InitPlatformPoseX()->get_RolledValue(),sfv_platPose->get_InitPlatformPoseY()->get_RolledValue(),plat_init_x, plat_init_y ,plat_init_z);
		plat_init_azi = sfv_platPose->get_InitPlatformPoseAzimut()->get_RolledValue();


		//set terrain bounds
		float xMax,yMax,xMin,yMin,temp;
		terrainA->getXYZCoord(0.95,0.95,xMax,yMax,temp);
		terrainA->getXYZCoord(0.05,0.05,xMin,yMin,temp);

		float wp_x=plat_init_x, wp_y=plat_init_y,  wp_azi = plat_init_azi;

		for (SFVwp *wp_it : *(sfv_Path->get_PathWPs()) )
			{
				wp_azi = wp_azi + wp_it->get_RalativeAngel()->get_RolledValue();
				wp_x = wp_x + wp_it->get_RalativeDistance()->get_RolledValue()*cos(wp_azi);
				wp_y = wp_y + wp_it->get_RalativeDistance()->get_RolledValue()*sin(wp_azi);

				if(wp_x<xMin || wp_y<yMin || wp_x>xMax || wp_y>yMax)
				{
					std::cout << " last wp : wp_x="<< wp_x <<" wp_y=" << wp_y << " is outside map bounds min_x="<< xMin <<" max_x=" << xMax <<" min_y="<< yMin <<" max_y=" << yMax << " - return false" << std::endl;
					return false; // last wp was outside map bounds
				}
			}
		std::cout << " all path wp are inside map bounds - return true" << std::endl;
		return true; // all the wp are inside map bounds
		}
}








