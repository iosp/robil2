/*
 * GazeboPlatformGenerator.cpp
 *
 *  Created on: Mar 27, 2014
 *      Author: userws1
 */

#include <Generators/Gazebo/GazeboPlatformGenerator.h>
#include <sdf/SDFImpl.hh>
#include <sdf/parser.hh>
#include <Resource/ResourceHandler.h>
#include <sdf/sdf.hh>
#include <boost/tokenizer.hpp>

#include "SFV/SFV.h"
#include "SFV/SFVmass_link.h"
#include "SFV/SFVfriction_link.h"
#include "SFV/SFVsensor_link.h"
#include "SFV/sfvSubGroup.h"

#include <stdlib.h>

GazeboPlatformGenerator::GazeboPlatformGenerator() {
	// TODO Auto-generated constructor stub

}

GazeboPlatformGenerator::~GazeboPlatformGenerator() {
	// TODO Auto-generated destructor stub
}

int findElementWithAttribute(sdf::ElementPtr *link, sdf::ElementPtr modelPtr,std::string elementType,std::string attributeType,std::string attributeName)
{
	sdf::ElementPtr res;
	for (res=modelPtr->GetElement(elementType) ; res /*->GetNextElement(elementType)*/ ; res=res->GetNextElement(elementType) )
	{
		if(res->HasAttribute(attributeType))
		{
			if(res->GetAttribute(attributeType)->GetAsString()==attributeName)
			{
			    *link = res;
				return 1;
			}
		}
	}
	return 0;
}



void GazeboPlatformGenerator::generatePlatform(SFV * sfv ,std::string filename, std::string nominal_platform_model_url, std::string scenario_models_folder_url)
{
	sdf::SDFPtr sdfptr(new sdf::SDF());
	init(sdfptr);
	sdf::addURIPath("model://", scenario_models_folder_url );
	sdf::readFile(nominal_platform_model_url,sdfptr);

	sdf::ElementPtr modelPtr=sdfptr->root->GetElement("model");
	sdf::ElementPtr link;


	// set mass links
	std::vector<SFVmass_link*> *massLinks_vec = new std::vector<SFVmass_link*>;
	sfv->get_VecOfSubGroupsByFeatureGroupType(ScenarioFeatureGroupType::mass_link_i, (std::vector<sfvSubGroup*> *)massLinks_vec);
	for(SFVmass_link * mass_link_it : * massLinks_vec )
	{
	if(findElementWithAttribute( &link, modelPtr,"link","name",mass_link_it->get_Name()))
		{

			sdf::ElementPtr inertial=link->GetElement("inertial");
			sdf::ElementPtr mass=inertial->GetElement("mass");
			sdf::ElementPtr inertia=inertial->GetElement("inertia");
			sdf::ElementPtr inertiaxx=inertia->GetElement("ixx");
			sdf::ElementPtr inertiayy=inertia->GetElement("iyy");
			sdf::ElementPtr inertiazz=inertia->GetElement("izz");
			sdf::ElementPtr pose=link->GetElement("pose");

			sdf::Pose new_pose;
			pose->GetValue()->Get(new_pose);
			new_pose.pos.x += mass_link_it->get_LocationDeviationX()->get_RolledValue();
			new_pose.pos.y += mass_link_it->get_LocationDeviationY()->get_RolledValue();
			new_pose.pos.z += mass_link_it->get_LocationDeviationZ()->get_RolledValue();
			sdf::Vector3 quad=new_pose.rot.GetAsEuler();
			quad.x += mass_link_it->get_LocationDeviationRoll()->get_RolledValue();
			quad.y += mass_link_it->get_LocationDeviationPitch()->get_RolledValue();
			quad.z += mass_link_it->get_LocationDeviationYaw()->get_RolledValue();
			new_pose.rot.SetFromEuler(quad);
			pose->Set(new_pose);


			double new_mass = mass->Get<double>() * mass_link_it->get_MassDeviation()->get_RolledValue();
			mass->Set(new_mass);

			double new_Ixx = inertiaxx->Get<double>() * mass_link_it->get_InertiaDeviationIxxComponent()->get_RolledValue();
			inertiaxx->Set(new_Ixx);

			double new_Iyy = inertiayy->Get<double>() * mass_link_it->get_InertiaDeviationIyyComponent()->get_RolledValue();
			inertiayy->Set(new_Iyy);

			double new_Izz = inertiazz->Get<double>() * mass_link_it->get_InertiaDeviationIzzComponent()->get_RolledValue();
			inertiazz->Set(new_Izz);
		}
		else
		{
			std::cout << mass_link_it->get_Name() << " mass link not found in sdf" << std::endl;
		}
	}

	// set friction links
	std::vector<SFVfriction_link*> *frictionLinks_vec = new std::vector<SFVfriction_link*>;
	sfv->get_VecOfSubGroupsByFeatureGroupType(ScenarioFeatureGroupType::friction_link_i, (std::vector<sfvSubGroup*> *)frictionLinks_vec);
	for(SFVfriction_link * friction_link_it : * frictionLinks_vec)
	{
		if(findElementWithAttribute( &link, modelPtr,"link","name",friction_link_it->get_Name()))
		{
			sdf::ElementPtr collision=link->GetElement("collision");
			sdf::ElementPtr surface=collision->GetElement("surface");
			sdf::ElementPtr friction=surface->GetElement("friction");
			sdf::ElementPtr ode=friction->GetElement("ode");
			sdf::ElementPtr mu=ode->GetElement("mu");
			sdf::ElementPtr mu2=ode->GetElement("mu2");

			double new_mu = mu->Get<double>() * friction_link_it->get_FrictionDeviation()->get_RolledValue();
			mu->Set<double>(new_mu);

			double new_mu2 = mu2->Get<double>() * friction_link_it->get_FrictionDeviation()->get_RolledValue();
			mu2->Set<double>(new_mu2);
		}
		else
		{
			std::cout << friction_link_it->get_Name() << " friction link not found in sdf" << std::endl;
		}
	}


	// set sensors links
	std::vector<SFVsensor_link*> *sensorsLinks_vec = new std::vector<SFVsensor_link*>;
	sfv->get_VecOfSubGroupsByFeatureGroupType(ScenarioFeatureGroupType::sensor_link_i, (std::vector<sfvSubGroup*> *)sensorsLinks_vec);
	for(SFVsensor_link * sensor_link_it : * sensorsLinks_vec)
	{
  		if(findElementWithAttribute( &link, modelPtr,"link","name",sensor_link_it->get_Name()))
		{
			sdf::ElementPtr pose=link->GetElement("pose");

			sdf::Pose new_pose;
			pose->GetValue()->Get(new_pose);
			new_pose.pos.x += sensor_link_it->get_LocationDeviationX()->get_RolledValue();
			new_pose.pos.y += sensor_link_it->get_LocationDeviationY()->get_RolledValue();
			new_pose.pos.z += sensor_link_it->get_LocationDeviationZ()->get_RolledValue();

			sdf::Vector3 quad=new_pose.rot.GetAsEuler();
			quad.x += sensor_link_it->get_LocationDeviationRoll()->get_RolledValue();
			quad.y += sensor_link_it->get_LocationDeviationPitch()->get_RolledValue();
			quad.z += sensor_link_it->get_LocationDeviationYaw()->get_RolledValue();
			new_pose.rot.SetFromEuler(quad);

			pose->GetValue()->Set(new_pose);
		}
		else
		{
			std::cout << sensor_link_it->get_Name() << " sensor link not found in sdf" << std::endl;
		}
	}

	sdfptr->Write(filename);
}



void GazeboPlatformGenerator::generate(SFV *sfv, std::string scenario_folder_url)
{
	std::string nominal_models_folder_url = ResourceHandler::getInstance(sfv->get_ResourceFile()).getRobotModelsFolderURL();
	std::string scenario_models_folder_url = scenario_folder_url + "/scenarioSystemModels" ;

	std::string create_scenario_models_folder_command = "mkdir -p " + scenario_models_folder_url;
	if ( system(create_scenario_models_folder_command.c_str()) )
	{ std::cout << "failed to : " << create_scenario_models_folder_command << std::endl; }

	std::string nominal_sensor_model_folder_url;
	std::string copy_sensor_folder_command;
	for (std::string * sensor_it : *(ResourceHandler::getInstance(sfv->get_ResourceFile()).getRobotSensorsNames()) )
	{
		nominal_sensor_model_folder_url = nominal_models_folder_url + "/" + *sensor_it;
		copy_sensor_folder_command = "cp -r " + nominal_sensor_model_folder_url + " " + scenario_models_folder_url;
		if ( system(copy_sensor_folder_command.c_str()) )
		{ std::cout << "failed to : " << copy_sensor_folder_command << std::endl; }
	}

	std::string nominal_platform_model_folder_url = nominal_models_folder_url + "/" +  ResourceHandler::getInstance(sfv->get_ResourceFile()).getRobotPlatformName() ;
	std::string scenario_platform_model_folder_url = scenario_models_folder_url + "/" + ResourceHandler::getInstance(sfv->get_ResourceFile()).getRobotPlatformName();

	std::string copy_platform_folder_command = "cp -r " + nominal_platform_model_folder_url + " " + scenario_models_folder_url;
	if ( system(copy_platform_folder_command.c_str()) )
		{ std::cout << "failed to : " << copy_sensor_folder_command << std::endl; }

	std::string nominal_platform_model_url = nominal_platform_model_folder_url + "/model.sdf";
	std::string scenario_platform_model_url = scenario_platform_model_folder_url + "/model.sdf";

	generatePlatform(sfv, scenario_platform_model_url ,nominal_platform_model_url, scenario_models_folder_url);


	std::cout << "\033[1;36m Producing " << scenario_platform_model_url << "\033[0m"<< std::endl;
}



