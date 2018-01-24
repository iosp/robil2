/*
 * RULE_platform_init_pose_with_no_obj_colisions.cpp
 *
 *  Created on: Jun 23, 2014
 *      Author: userws3
 */


#include "Rules/RULE_platform_init_pose_with_no_obj_colisions.h"
#include "SFV/SFV.h"
#include "Resource/ResourceHandler.h"
#include "Generators/Gazebo/TerrainAnalyzer.h"

#include "SFV/SFVplatformPose.h"
#include "SFV/SFVobjScattering.h"
#include "SFV/SFVterraine.h"

#include <math.h>

#include <iostream>
#include <vector>

Rule_platform_init_pose_with_no_obj_colisions::Rule_platform_init_pose_with_no_obj_colisions() {
	// TODO Auto-generated constructor stub
}

Rule_platform_init_pose_with_no_obj_colisions::~Rule_platform_init_pose_with_no_obj_colisions() {
	// TODO Auto-generated destructor stub
}


bool Rule_platform_init_pose_with_no_obj_colisions::isRuleValid(SFV *sfv)
{
		std::cout << " !! checking Rule_platform_init_pose_with_no_obj_colisions  --- ";

		SFVplatformPose *sfv_PlatPose = (SFVplatformPose*)(sfv->get_SubGroupByFeatureGroupType(ScenarioFeatureGroupType::platform_pose));

		if (! sfv_PlatPose->get_WasRolledFlag())
			{
			std::cout << " platform initial position hasn't been rolled yet, so the rule have no meaning for now  - return true" << std::endl;
			return true;
			}


		std::vector<SFVobjScattering*> *objects_scatterings_vec = new std::vector<SFVobjScattering*>;
   	   	sfv->get_VecOfSubGroupsByFeatureGroupType(ScenarioFeatureGroupType::objects, (std::vector<sfvSubGroup*> *)objects_scatterings_vec);
		if (objects_scatterings_vec->empty())
			{
			std::cout << " no Object was rolled yet, so the rule has no meaning for now  - return true" << std::endl;
			return true;
			}


		SFVterraine *sfv_terrain = (SFVterraine*)(sfv->get_SubGroupByFeatureGroupType(ScenarioFeatureGroupType::map));
		if (! sfv_terrain->get_WasRolledFlag())
			{
			std::cout << " terrain hasn't been rolled yet, so the rule can't be evaluated - return false" << std::endl;
			return false; // terrain is needed for translation of percents to distance
			}

		//load terrain
		std::string terrain_name=ResourceHandler::getInstance(sfv->get_ResourceFile()).getTerrainById(sfv_terrain->get_TopographicMapIndex()->get_RolledValue());
		std::string teraine_file_url = ResourceHandler::getInstance(sfv->get_ResourceFile()).getWorldModelsFolderURL();
		TerrainAnalyzer* terrainA=new TerrainAnalyzer();
		terrainA->loadFile(teraine_file_url+"/"+terrain_name);

		//get platform initial position
		float plat_init_x , plat_init_y, plat_init_z;
		terrainA->getXYZCoord(sfv_PlatPose->get_InitPlatformPoseX()->get_RolledValue(),sfv_PlatPose->get_InitPlatformPoseY()->get_RolledValue(),plat_init_x, plat_init_y ,plat_init_z);

		for (SFVobjScattering* obj_scat_it : * objects_scatterings_vec )
		{
			//std::cout << " obj_scat_it->get_NumberOfObjects()->get_RolledValue() = " << obj_scat_it->get_NumberOfObjects()->get_RolledValue() << std::endl;
			if ( obj_scat_it->get_NumberOfObjects()->get_RolledValue() > 0 )
			{
				for (SFVObject *obj : *(obj_scat_it->get_Objects()))
				{
				// TODO platform and objects orientation shall be also taken to consideration

					float obj_x, obj_y, obj_z;
					terrainA->getXYZCoord(obj->get_LocationOnTheXaxis()->get_RolledValue(),obj->get_LocationOnTheYaxis()->get_RolledValue(),obj_x, obj_y ,obj_z);

					float dx = plat_init_x - obj_x;
					float dy = plat_init_y - obj_y;

					float dis = std::sqrt(std::pow(dx,2) + std::pow(dy,2));

					if(dis <= 2.5)
					{
						std::cout << " object at obj_x="<< obj_x <<" obj_y=" << obj_y << " is closer than 2.5m to the platform at plat_x="<< plat_init_x <<" plat_y=" << plat_init_y <<" - return false" << std::endl;
						return false; // obj is closer than 5m to the platform
					}
				}
			}
		}
		std::cout << " all the obj are in distance of more than 5m from the bobcat - return true" << std::endl;
		return true; // all the obj are in distance of more than 5m from the bobcat
}








