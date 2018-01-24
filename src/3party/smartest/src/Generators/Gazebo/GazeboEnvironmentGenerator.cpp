/*
 * EnvironmentGenerator.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: userws1
 */

#include "Generators/Gazebo/GazeboEnvironmentGenerator.h"
#include "Generators/Gazebo/TerrainAnalyzer.h"

#include "SFV/SFV.h"
#include "SFV/SFVobjScattering.h"
#include "SFV/SFVterraine.h"
#include "SFV/SFVplatformPose.h"
#include "SFV/SFVpath.h"
#include "SFV/SFVobsOnPathScattering.h"

#include "utils/TinyXmlDef.h"
#include "Resource/ResourceHandler.h"
#include <sstream>
#include <stdlib.h>
#include <sdf/sdf.hh>

#include <iostream>

GazeboEnvironmentGenerator::GazeboEnvironmentGenerator() {
	 m_terrainAnalyzer=new TerrainAnalyzer();
	 m_objectCount=0;
	 m_ObstacleOnPathCounter=0;
	 m_WpMarkCounter=0;
}

GazeboEnvironmentGenerator::~GazeboEnvironmentGenerator() {
	// TODO Auto-generated destructor stub
}
void GazeboEnvironmentGenerator::spawnObjects(SFV* sfv,TiXmlElement * element)
{
	std::vector<SFVobjScattering*> *objects_scatterings_vec = new std::vector<SFVobjScattering*>;
	if (! ( sfv->get_VecOfSubGroupsByFeatureGroupType(ScenarioFeatureGroupType::objects, (std::vector<sfvSubGroup*> *)objects_scatterings_vec)) )
		{ return; }

	for (SFVobjScattering* obj_scat_it : * objects_scatterings_vec )
	{
		if ( obj_scat_it->get_NumberOfObjects()->get_RolledValue() > 0 )
		{
			for (SFVObject* obj : *obj_scat_it->get_Objects())
			{
				TiXmlElement * include = new TiXmlElement( "include" );
				element->LinkEndChild(include);
				TiXmlElement * url = new TiXmlElement( "uri" );
				TiXmlText * url_text = new TiXmlText(std::string("model://")+ResourceHandler::getInstance(sfv->get_ResourceFile()).getObjectById(obj->get_ObjectType()->get_RolledValue()));
				url->LinkEndChild(url_text);
				include->LinkEndChild(url);
				TiXmlElement * pose = new TiXmlElement( "pose" );
				std::stringstream ss;

				float obj_x = obj->get_Object_xyz()->at('x');
				float obj_y = obj->get_Object_xyz()->at('y');
				float obj_z = obj->get_Object_xyz()->at('z');

				ss<<obj_x <<" " << obj_y <<" "<< obj_z << " ";
				ss<<obj->get_LocationRoll()->get_RolledValue()<<" " << obj->get_LocationPitch()->get_RolledValue()<<" "<< obj->get_LocationYaw()->get_RolledValue();
				TiXmlText * pose_text = new TiXmlText(ss.str());
				ss.str("");
				pose->LinkEndChild(pose_text);
				include->LinkEndChild(pose);
				TiXmlElement * scale = new TiXmlElement( "scale" );

				ss<<obj->get_ScalingFactor()->get_RolledValue() <<" " << obj->get_ScalingFactor()->get_RolledValue() <<" "<< obj->get_ScalingFactor()->get_RolledValue();
				TiXmlText * scale_text = new TiXmlText(ss.str());
				ss.str("");
				scale->LinkEndChild(scale_text);
				include->LinkEndChild(scale);
				TiXmlElement * name = new TiXmlElement( "name" );
				include->LinkEndChild(name);
				ss.str("");
				ss<< "object";
				ss<<m_objectCount;
				name->LinkEndChild(new TiXmlText(ss.str()));
				m_objectCount++;
			}
		}
	}
}


void GazeboEnvironmentGenerator::spawnObstacleOnPath(SFV* sfv,TiXmlElement * element)
{
	std::vector<SFVobsOnPathScattering*> *obsOnpath_scatterings_vec = new std::vector<SFVobsOnPathScattering*>;
	if ( ! (sfv->get_VecOfSubGroupsByFeatureGroupType(ScenarioFeatureGroupType::obstacles_on_path, (std::vector<sfvSubGroup*> *)obsOnpath_scatterings_vec) ) )
	 { return; }

	for (SFVobsOnPathScattering *obsScattering_it : *obsOnpath_scatterings_vec )
	{
		if ( obsScattering_it->get_NumberOfObstaclesOnPath()->get_RolledValue() > 0 )
		{
			for (SFVObstacleOnPath *obs : *(obsScattering_it->get_ObstaclesOnPath()))
			{
				double obs_x = obs->get_Obstacle_xyz()->at('x');
				double obs_y = obs->get_Obstacle_xyz()->at('y');
				double obs_z = obs->get_Obstacle_xyz()->at('z');


				std::stringstream ss;
				ss<<obs_x <<" " << obs_y <<" "<< obs_z << " ";
				ss<<obs->get_LocationRoll()->get_RolledValue() <<" " << obs->get_LocationPitch()->get_RolledValue() <<" "<< obs->get_LocationYaw()->get_RolledValue();


				TiXmlElement * include = new TiXmlElement( "include" );
				element->LinkEndChild(include);
				TiXmlElement * url = new TiXmlElement( "uri" );
				TiXmlText * url_text = new TiXmlText(std::string("model://")+ResourceHandler::getInstance(sfv->get_ResourceFile()).getObjectById(obs->get_ObstacleType()->get_RolledValue()));
				url->LinkEndChild(url_text);
				include->LinkEndChild(url);



				TiXmlText * pose_text = new TiXmlText(ss.str());
				ss.str("");
				TiXmlElement * pose = new TiXmlElement( "pose" );
				pose->LinkEndChild(pose_text);
				include->LinkEndChild(pose);


				TiXmlElement * scale = new TiXmlElement( "scale" );
				ss<<obs->get_ScalingFactor()->get_RolledValue() <<" " << obs->get_ScalingFactor()->get_RolledValue() <<" "<< obs->get_ScalingFactor()->get_RolledValue();
				TiXmlText * scale_text = new TiXmlText(ss.str());
				ss.str("");
				scale->LinkEndChild(scale_text);
				include->LinkEndChild(scale);

				TiXmlElement * name = new TiXmlElement( "name" );
				include->LinkEndChild(name);
				ss.str("");
				ss<< "obstacle_on_path";
				ss<<m_ObstacleOnPathCounter;

				name->LinkEndChild(new TiXmlText(ss.str()));
				m_ObstacleOnPathCounter++;
			}
		}
	}
}




void GazeboEnvironmentGenerator::spawnPathWpMarks(SFV* sfv,TiXmlElement * element)
{
	SFVpath *sfv_Path = (SFVpath*)(sfv->get_SubGroupByFeatureGroupType(ScenarioFeatureGroupType::Path));

    double wp_x, wp_y;
	for(SFVwp* wp_it : *(sfv_Path->get_PathWPs()))
	     {

	    wp_x = wp_it->get_WPxy()->at('x');
	    wp_y = wp_it->get_WPxy()->at('y');


	    TiXmlElement * include = new TiXmlElement( "include" );
		element->LinkEndChild(include);
		TiXmlElement * url = new TiXmlElement( "uri" );

		TiXmlText * url_text = new TiXmlText("model://wp_mark");
		url->LinkEndChild(url_text);
		include->LinkEndChild(url);

		TiXmlElement * pose = new TiXmlElement( "pose" );
		std::stringstream ss;
		float mark_x = wp_x, mark_y = wp_y, mark_z;
		m_terrainAnalyzer->getZCoord(mark_x,mark_y,mark_z);
		ss<<mark_x <<" " << mark_y <<" "<< mark_z+0 << " ";
		ss<<0 <<" " << 0 <<" "<< 0;
		TiXmlText * pose_text = new TiXmlText(ss.str());
		ss.str("");
		pose->LinkEndChild(pose_text);
		include->LinkEndChild(pose);

		TiXmlElement * name = new TiXmlElement( "name" );
		include->LinkEndChild(name);
		ss.str("");
		ss<< "wp_mark";
		ss<<m_WpMarkCounter;

		name->LinkEndChild(new TiXmlText(ss.str()));
		m_WpMarkCounter++;
	     }
}



void GazeboEnvironmentGenerator::spawnTerrain(SFV* sfv,TiXmlElement * element)
{
	SFVterraine *sfv_terraine = (SFVterraine*)(sfv->get_SubGroupByFeatureGroupType(ScenarioFeatureGroupType::map));
	std::string terrain=ResourceHandler::getInstance(sfv->get_ResourceFile()).getTerrainById(sfv_terraine->get_TopographicMapIndex()->get_RolledValue());

	TiXmlElement * include = new TiXmlElement( "include" );
	element->LinkEndChild(include);
	TiXmlElement * url = new TiXmlElement( "uri" );
	TiXmlText * url_text = new TiXmlText(std::string("model://") + terrain);
	url->LinkEndChild(url_text);
	include->LinkEndChild(url);
}


void GazeboEnvironmentGenerator::spawnPlatformPose(SFV* sfv,TiXmlElement * element)
{
	SFVplatformPose *sfv_PlatPose = (SFVplatformPose*)(sfv->get_SubGroupByFeatureGroupType(ScenarioFeatureGroupType::platform_pose));

	TiXmlElement * include = new TiXmlElement( "include" );
	element->LinkEndChild(include);
	TiXmlElement * url = new TiXmlElement( "uri" );
	TiXmlText * url_text = new TiXmlText(std::string("model://")+ResourceHandler::getInstance(sfv->get_ResourceFile()).getRobotPlatformName());
	url->LinkEndChild(url_text);
	include->LinkEndChild(url);
	TiXmlElement * pose = new TiXmlElement( "pose" );
	std::stringstream ss;
	float x,y,z, azi;
	m_terrainAnalyzer->getXYZCoord(sfv_PlatPose->get_InitPlatformPoseX()->get_RolledValue(),sfv_PlatPose->get_InitPlatformPoseY()->get_RolledValue(),x,y,z);
    azi = sfv_PlatPose->get_InitPlatformPoseAzimut()->get_RolledValue();

	z=z+0.5;  // compensation for the wheels height
	ss<<x <<" " << y <<" "<< z << " ";
	ss<<0 <<" " << 0 <<" "<< azi;
	TiXmlText * pose_text = new TiXmlText(ss.str());
	ss.str("");
	pose->LinkEndChild(pose_text);
	include->LinkEndChild(pose);

	TiXmlElement * name = new TiXmlElement( "name" );
	name->LinkEndChild(new TiXmlText(ResourceHandler::getInstance(sfv->get_ResourceFile()).getRobotName()));
	include->LinkEndChild(name);
}



void GazeboEnvironmentGenerator::genEnvFromSFV(SFV* sfv,std::string filename)
{
	//load terrain
	SFVterraine *sfv_terraine = (SFVterraine*)(sfv->get_SubGroupByFeatureGroupType(ScenarioFeatureGroupType::map));
	std::string terrain=ResourceHandler::getInstance(sfv->get_ResourceFile()).getTerrainById(sfv_terraine->get_TopographicMapIndex()->get_RolledValue());

	std::string path = ResourceHandler::getInstance(sfv->get_ResourceFile()).getWorldModelsFolderURL();

	m_terrainAnalyzer->loadFile(path+"/"+terrain);

	TiXmlDocument doc(filename);
	TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
	doc.LinkEndChild(decl);
	TiXmlElement * element = new TiXmlElement( "sdf" );
	element->SetAttribute("version","1.4");
	doc.LinkEndChild(element);
	TiXmlElement * world = new TiXmlElement( "world" );
	world->SetAttribute("name","default");
	element->LinkEndChild(world);


	// addition of sunlight
	TiXmlElement * include = new TiXmlElement( "include" );
	world->LinkEndChild(include);
	TiXmlElement * uri = new TiXmlElement( "uri" );
	TiXmlText * sun_url = new TiXmlText("model://sun");
	uri->LinkEndChild(sun_url);
	include->LinkEndChild(uri);

	// addition of sky to the environment
	TiXmlElement * scene = new TiXmlElement( "scene" );
	world->LinkEndChild(scene);
	TiXmlElement * sky = new TiXmlElement( "sky" );
	scene->LinkEndChild(sky);

	TiXmlElement * time = new TiXmlElement( "time" );
	sky->LinkEndChild(time);
	TiXmlText * time_value = new TiXmlText("10");
	time->LinkEndChild(time_value);

	// addition of clouds
	TiXmlElement * clouds = new TiXmlElement( "clouds" );
	sky->LinkEndChild(clouds);

	TiXmlElement * speed = new TiXmlElement( "speed" );
	clouds->LinkEndChild(speed);
	TiXmlText * speed_value = new TiXmlText("5");
	speed->LinkEndChild(speed_value);

	TiXmlElement * direction = new TiXmlElement( "direction" );
	clouds->LinkEndChild(direction);
	TiXmlText * direction_value = new TiXmlText("1.14");
	direction->LinkEndChild(direction_value);
	TiXmlElement * humidity = new TiXmlElement( "humidity" );
	clouds->LinkEndChild(humidity);
	TiXmlText * humidity_value = new TiXmlText("0.3");
	humidity->LinkEndChild(humidity_value);

	TiXmlElement * gui = new TiXmlElement( "gui" );
	world->LinkEndChild(gui);
	TiXmlElement * camera = new TiXmlElement( "camera" );
	camera->SetAttribute("name","camera_pose");
	gui->LinkEndChild(camera);
	TiXmlElement * camera_pose = new TiXmlElement( "pose" );
	camera->LinkEndChild(camera_pose);
	TiXmlText * camera_pose_value = new TiXmlText("80 -60 60 0 0.5 2");
	camera_pose->LinkEndChild(camera_pose_value);

	spawnTerrain(sfv, world);

	spawnPlatformPose(sfv, world);

	spawnObjects(sfv, world);
	spawnObstacleOnPath(sfv, world);

	spawnPathWpMarks(sfv, world);

	doc.SaveFile(filename.c_str());
}

void GazeboEnvironmentGenerator::generate(SFV* sfv, std::string scenario_folder_url)
{
	std::string temp = scenario_folder_url+"/scenarioEnv.world";
	std::cout << "\033[1;36m Producing " << temp << "\033[0m"<< std::endl;
	genEnvFromSFV(sfv, temp);
}

