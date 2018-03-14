/*
 * SFV.cpp
 *
 *  Created on: Jul 30, 2014
 *      Author: userws3
 */

#include "SFV/SFV.h"
//#include "SFDP/SFDPobj.h"
#include "SFV/sfvSubGroup.h"
#include "SFV/SFVpath.h"
#include "SFV/SFVmass_link.h"
#include "SFV/SFVsensor_link.h"
#include "SFV/SFVfriction_link.h"
#include "SFV/SFVterraine.h"
#include "SFV/SFVobjScattering.h"
#include "SFV/SFVplatformPose.h"
#include "SFV/SFVobsOnPathScattering.h"

#include "SFDP/ScenarioFeatureGroup.h"
#include "SFDP/ScenarioFeatureGroupType.h"

#include "Rules/RULE_platform_init_pose_with_no_obj_colisions.h"
#include "Rules/RULE_wp_path_inside_map.h"

#include <tinyxml.h>
#include "utils/TinyXmlDef.h"

#include "Generators/Gazebo/GazeboScenarioGenerator.h"
#include "Executor/GazeboExecutor.h"

#include "std_msgs/Float32MultiArray.h"


class sfvSubGroup;
SFV::SFV(SFDPobj * SFDP, std::string ws_folder_url)
{
	my_SFDP = SFDP;
	my_resource_file_url = SFDP->get_Resources_file_url();
	my_ws_folder_url = ws_folder_url;
	was_rolled_flag = false;
	was_executed_flag = false;

	my_sfvSubGroups = new std::vector<sfvSubGroup *>;
	for (ScenarioFeatureGroup * featureGroup_it : * (my_SFDP->get_FeatureGroups()) )
		{
		Populate_mySFVsubGroups<ScenarioFeatureGroup *>(featureGroup_it->get_featureGroupType(), featureGroup_it);
		}

	my_rules = new std::vector<Rule *>;
	my_rules->push_back(new Rule_platform_init_pose_with_no_obj_colisions);
	my_rules->push_back(new Rule_wp_path_inside_map);

	SubGroupsBayFeatureGroupType_ReturnVec = new std::vector<sfvSubGroup *>;
}


SFV::SFV(std::string SFV_file_name, std::string ws_folder_url)
{
	my_SFDP = 0;
	my_ws_folder_url = ws_folder_url;
	was_rolled_flag=true;
	my_rules=0;
	my_sfvSubGroups = new std::vector<sfvSubGroup *>;
	SubGroupsBayFeatureGroupType_ReturnVec = new std::vector<sfvSubGroup *>;

	//Loading file
	TiXmlDocument *SFVfile = new TiXmlDocument(SFV_file_name);
	if (!SFVfile->LoadFile())
	{
		std::cout << "\033[1;31m Failed to load file : " << SFV_file_name << " it might not exist or be not valid XML \033[0m" << std::endl;
	}

	//Search for an sfv element to parse
	TiXmlNode* sfv_xml = 0 ;
	for ( sfv_xml = SFVfile->FirstChild(); sfv_xml != 0; sfv_xml = sfv_xml->NextSibling())
		{
		if(sfv_xml->Type()==XML_ELEMENT && sfv_xml->ValueStr().compare("SFV")==0)
			{ break; }
		}
	if (sfv_xml == 0)
	{
		std::cout << "\033[1;31m There is no SFV xml element in file : " << SFV_file_name << "\033[0m" << std::endl;
	}

	//search for an ResourceFileURL attribute to parse
	TiXmlAttribute* sfv_xml_att;
	for ( sfv_xml_att = sfv_xml->ToElement()->FirstAttribute(); sfv_xml_att != 0; sfv_xml_att = sfv_xml_att->Next())
		{
		if(sfv_xml_att->NameTStr().compare("ResourceFileURL")==0)
			{
			my_resource_file_url = sfv_xml_att->ValueStr();
			break;
			}
		}
	if (sfv_xml_att == 0)
	{
		std::cout << "\033[1;31m There is no ResourceFileURL attribute in file : " << SFV_file_name << "\033[0m" << std::endl;
	}

	//Parsing by Features Groups xml elements
	TiXmlNode* featureGroup_xml = 0 ;
	for ( featureGroup_xml = sfv_xml->FirstChild(); featureGroup_xml != 0; featureGroup_xml = featureGroup_xml->NextSibling())
		{
		if (featureGroup_xml->Type()==XML_ELEMENT)
			{
			Populate_mySFVsubGroups<TiXmlNode*>(ScenarioFeatureGroupType::get_by_name(featureGroup_xml->ValueStr().c_str()).get(), featureGroup_xml);
			}
		}
	std::cout << "my_resource_file_url = " << my_resource_file_url <<std::endl;
}


template <class T>
void SFV::Populate_mySFVsubGroups(ScenarioFeatureGroupType::optional scenarioFeatureType , T fatures_data)
{
	 switch (scenarioFeatureType.get().value())
	 	 {
	 	 	 case(ScenarioFeatureGroupType::map) :
				my_sfvSubGroups->push_back( new SFVterraine(fatures_data, this) );
		 		break;

		 	 case(ScenarioFeatureGroupType::objects) :
				my_sfvSubGroups->push_back( new SFVobjScattering(fatures_data, this) );
		 	 	break;

		 	 case(ScenarioFeatureGroupType::platform_pose) :
				my_sfvSubGroups->push_back( new SFVplatformPose(fatures_data, this) );
		 	 	break;

		 	 case(ScenarioFeatureGroupType::Path) :
				my_sfvSubGroups->push_back( new SFVpath(fatures_data, this) );
		 		break;

		 	 case(ScenarioFeatureGroupType::obstacles_on_path) :
				my_sfvSubGroups->push_back( new SFVobsOnPathScattering(fatures_data, this) );
		 		break;

		 	 case(ScenarioFeatureGroupType::mass_link_i) :
				my_sfvSubGroups->push_back( new SFVmass_link(fatures_data, this) );
		 		break;

		 	 case(ScenarioFeatureGroupType::friction_link_i) :
				my_sfvSubGroups->push_back( new SFVfriction_link(fatures_data, this) );
		 		break;

		 	 case(ScenarioFeatureGroupType::sensor_link_i) :
				my_sfvSubGroups->push_back( new SFVsensor_link(fatures_data, this) );
		 		break;
	 	 }
}


sfvSubGroup * SFV::get_SubGroupByFeatureGroupType(ScenarioFeatureGroupType GroupType)
{
	for (sfvSubGroup * subGroup_it : * my_sfvSubGroups)
		{
		if (subGroup_it->get_Type().value() == GroupType.value())
			{
			return(subGroup_it);
			}
		}
}

bool SFV::get_VecOfSubGroupsByFeatureGroupType(ScenarioFeatureGroupType GroupType, std::vector<sfvSubGroup *> * SubGroupVec)
{
	for (sfvSubGroup * subGroup_it : * my_sfvSubGroups)
		{
		if (subGroup_it->get_Type().value() == GroupType.value())
			{
			SubGroupVec->push_back(subGroup_it);
			}
		}

	if (SubGroupVec->empty())
		return(false);
	else
		return(true);
}


bool SFV::roll()
{
	if (was_rolled_flag)
	{
		std::cout << "\033[1;31m I already was rolled (I am SFV) \033[0m"<< std::endl;
		return(false);
	}
	else
	{
		int roll_attemps_limit = 3;
		int roll_attemp = 1;
		bool roll_fail_flag = false;

		while (roll_attemp <= roll_attemps_limit)
			{
			for (sfvSubGroup * subGroup_it : * my_sfvSubGroups)
				{
				std::cout << "Roll of " << subGroup_it->get_Type().str() <<std::endl;
				if (! subGroup_it->roll())
					{
					roll_fail_flag=true;
					break;
					}
				}

			if (roll_fail_flag)
				{
				my_sfvSubGroups = new std::vector<sfvSubGroup *>;
				for (ScenarioFeatureGroup * featureGroup_it : * (my_SFDP->get_FeatureGroups()) )
					{
					Populate_mySFVsubGroups<ScenarioFeatureGroup *>(featureGroup_it->get_featureGroupType(), featureGroup_it);
					}

				roll_fail_flag=false;
				std::cout << "\033[1;35m Fail to roll SFV attempt = " << roll_attemp << " / " << roll_attemps_limit << "\033[0m"<< std::endl;
				}
			else
				{
				was_rolled_flag = true;
				std::cout << "\033[1;32m Succeed to roll SFV attempt = " << roll_attemp << " / " << roll_attemps_limit << "\033[0m"<< std::endl;
				return(true);
				}
			roll_attemp++;
			}
		return(false);
	}
}


bool SFV::rules_check()
{
	for (Rule * rule_it : * my_rules)
	{
	 if(! rule_it->isRuleValid(this))
    	 {
		 return(false);
	     }
	}
	return(true);
}



int SFV::printToXML(std::string sfv_file_url)
{
	if (! was_rolled_flag)
	{
		std::cout << "\033[1;31m can't print SFV because it wasn't rolled yet \033[0m"<< std::endl;
		return(0);
	}
	else
	{
		TiXmlElement * xml_SFV = new TiXmlElement("SFV");
		xml_SFV->SetAttribute("ID",std::to_string(1));
		xml_SFV->SetAttribute("ResourceFileURL" , my_resource_file_url);

		TiXmlElement * xml_subGroup;
		int id=1;
		for (sfvSubGroup * subGroup_it : * get_sfvSubGroups())
		{
			xml_subGroup = subGroup_it->ToXmlElement(id);
			if (xml_subGroup)
			{
			xml_SFV->LinkEndChild(xml_subGroup);
			id++;
			}
		}

		TiXmlDocument doc(sfv_file_url);
		TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
		doc.LinkEndChild(decl);

		doc.LinkEndChild(xml_SFV);
		doc.SaveFile(sfv_file_url.c_str());

		std::cout << " printing to file : " << sfv_file_url << std::endl;

		return(1);
	}
}

int SFV::generate()
{
	GazeboScenarioGenerator * ScenGen = new GazeboScenarioGenerator(this, my_ws_folder_url);
	if (! ScenGen->GenerateScenario())
	{
		std::cout << "\033[1;31m Generation of SFVs scenario files have failed \033[0m"<< std::endl;
		return(0);
	}

	return 1;
}

int SFV::execute(int argc, char** argv)
{
	generate();

	GazeboExecutor * ScenExe = new GazeboExecutor(this);
	if (! ScenExe->RunScenario(argc,argv))
	{
		std::cout << "\033[1;31m Execution of SFVs scenario have failed \033[0m"<< std::endl;
		return(0);
	}

	my_grades = ScenExe->get_scenario_grades();
	was_executed_flag = true;
	PrintResultsToFile();

	return 1;
}


TiXmlElement *SFV::get_GradesAsXMLElement(int sfv_index)
{
	if (! was_executed_flag)
	{
		std::cout << " \033[1;31m can't create Grades XML element because the SFV havn't been executed yet \033[0m" << std::endl;
		return 0;
	}

	TiXmlElement * xml_grades = new TiXmlElement( "Scenario_" + std::to_string(sfv_index) );

	std::stringstream temp_ss;
	for (int i=0  ; i < my_grades->data.size() ; i++)
	{
		temp_ss.str(" ");
		temp_ss << my_grades->data[i];
		TiXmlElement * xml_grade = new TiXmlElement( "grade_" + std::to_string(i) );
		TiXmlText * xml_grade_val = new TiXmlText( temp_ss.str() );
		xml_grade->LinkEndChild(xml_grade_val);
		xml_grades->LinkEndChild(xml_grade);

		temp_ss << " ";
	}
	return(xml_grades);
}


int SFV::PrintResultsToFile()
{
	if (! was_executed_flag)
	{
		std::cout << " \033[1;31m can't print results file because the SFV havn't been executed yet \033[0m" << std::endl;
		return 0;
	}

	std::string my_Grades_file_url = get_WSfolder() + "/SFV_execution_grades.xml";

	TiXmlDocument doc(my_Grades_file_url);
	TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
	doc.LinkEndChild(decl);

	TiXmlElement * xml_results = new TiXmlElement("SFV_execution_grades");
	xml_results->SetAttribute("version","1.0");
	doc.LinkEndChild(xml_results);

	TiXmlElement * xml_grades = get_GradesAsXMLElement(0);
	doc.LinkEndChild(xml_grades);

	doc.SaveFile(my_Grades_file_url.c_str());
	std::cout << " printing Grades to file : " << my_Grades_file_url << std::endl;

	return 1;
}



SFV::~SFV()
{

}

