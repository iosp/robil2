/*
 * SFDPobj.cpp
 *
 *  Created on: Jul 1, 2014
 *      Author: userws3
 */

#include "SFDP/SFDPobj.h"
#include "SFV/SFV.h"
//#include "Generators/Gazebo/GazeboScenarioGenerator.h"
//#include "Executor/GazeboExecutor.h"

#include <string>
#include <vector>

#include <iostream>

#include <boost/filesystem.hpp>

#include <cmath> //sqrt()

#include <tinyxml.h>
#include "utils/TinyXmlDef.h"



SFDPobj::SFDPobj(std::string SFDP_file_url, std::string Resources_file_url, std::string WS_url, int division_level)
{
	my_SFDP_file_url = SFDP_file_url;
	my_Resources_file_url = Resources_file_url;
	my_WS_url = WS_url;
	my_Grades_file_url = WS_url + "Grades.xml";
	my_division_level = division_level;
	have_been_run = false;

	my_featureGroups = new std::vector<ScenarioFeatureGroup*>;
	my_sampled_SFVs = new std::vector<SFV *>;
	my_sub_SFDPs = new std::vector<SFDPobj *>;

	my_ExploretionFeature = new ScenarioFeature();

	my_Grades_means = new std::vector<float>;
	my_Grades_stds = new std::vector<float>;
}


int SFDPobj::ParseMeFromXMLFile()
{
	TiXmlDocument *XMLfile = new TiXmlDocument(my_SFDP_file_url);
	if (!XMLfile->LoadFile())
	{
		std::cout << "\033[1;31m Failed to load file = " << my_SFDP_file_url << " it might not exist or be not valid XML \033[0m" << std::endl;
		return 0;
	}

	TiXmlNode* XMLfile_Child;
	for ( XMLfile_Child = XMLfile->FirstChild() ; XMLfile_Child != 0; XMLfile_Child = XMLfile_Child->NextSibling())
	{
		if ( (XMLfile_Child->Type()==XML_ELEMENT) && (XMLfile_Child->ValueStr().compare("sfdp")==0) )
		{
			break;
		}
	}

	if (XMLfile_Child == 0)
	{
		std::cout << "\033[1;31m Failed to parse SFDP file = " << my_SFDP_file_url << " it has no SFDP element \033[0m" << std::endl;
		return 0;
	}

	TiXmlNode* SFDP_Child;
	for ( SFDP_Child = XMLfile_Child->FirstChild() ; SFDP_Child != 0; SFDP_Child = SFDP_Child->NextSibling())
		{
			ScenarioFeatureGroup *featureGroup=new ScenarioFeatureGroup();
			if ( featureGroup->parseScenarioFeatureGroupFromXML(SFDP_Child) )
				{
				my_featureGroups->push_back(featureGroup);
				}
			else
				{
				std::cout << "\033[1;31m Failed to parse SFDP file = " << my_SFDP_file_url  << " one or more of it's Scenario Feature Groups is not valid \033[0m" << std::endl;
				return 0;
				}
		}
	return 1;
}



int SFDPobj::PrintMeToFile()
{
	TiXmlElement * xml_SFDP = new TiXmlElement( "SFDP" );
	xml_SFDP->SetAttribute("version","1.0");

	for (ScenarioFeatureGroup * group_it : * my_featureGroups )
		{
		xml_SFDP->LinkEndChild(group_it->toXMLElement());
		}


	TiXmlDocument doc(my_SFDP_file_url);
	TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
	doc.LinkEndChild(decl);

	doc.LinkEndChild(xml_SFDP);
	doc.SaveFile(my_SFDP_file_url.c_str());

	std::cout << " printing to file : " << my_SFDP_file_url << std::endl;

	return 0;
}



int SFDPobj::GenMySFVs(int samp_num)
{
	my_sampled_SFVs = new std::vector<SFV *>;

	SFV * sfv_temp;
	std::string file_url;

	int success_num=0;
	int sfv_index=1;

	for (int sfv_index=1 ; sfv_index<=samp_num ; sfv_index++ )
	{

		std::string folder_url = my_WS_url + "sampl_" + std::to_string(sfv_index);
		file_url = folder_url + "/scen.SFV";


		boost::filesystem::remove_all(folder_url);
		if(! boost::filesystem::create_directory(folder_url))
			{
			std::cout << "\033[1;31m failed to create folder for sfv_" << sfv_index << "\033[0m"<< std::endl;
			break;
			}

		sfv_temp = new SFV(this,folder_url);
		if ( ! sfv_temp )
			{
			std::cout << "\033[1;31m failed to Generate sfv_ " << sfv_index << "\033[0m" <<std::endl;
			break;
			}

		if (! sfv_temp->roll() )
			{
			std::cout << "\033[1;31m failed to Roll sfv_ " << sfv_index << "\033[0m" <<std::endl;
			break;
			}

			my_sampled_SFVs->push_back(sfv_temp);
			success_num++;
			sfv_temp->printToXML(file_url);
	}

	std::cout << "success in rolling " << success_num << "/" << samp_num << " SFVs " << std::endl;

	if (success_num == samp_num)
		return 1;
	else
		return 0;
}




int SFDPobj::RunMySFVs(int argc, char** argv)
{
	int sfv_index=0;
	std::vector<float> grads_sums;
	std::vector<float> grads_sums_of_squers;

	for (SFV * sfv_it : * my_sampled_SFVs )
	{
		sfv_it->execute(argc, argv);
		if (sfv_it->get_WasExecutedFlag())
			{
			for(int i=0 ; i<(sfv_it->get_Grades())->data.size() ; i++)
				{
					float grade = sfv_it->get_Grades()->data[i];
					if (grads_sums.size()<=i)
						{
						float new_grades_sum = grade;
						grads_sums.push_back(new_grades_sum);
						float new_grades_sqr = grade*grade;
						grads_sums_of_squers.push_back(new_grades_sqr);
						}
					else
						{
						grads_sums[i] = grads_sums[i] + grade;
						grads_sums_of_squers[i] = grads_sums_of_squers[i] + grade*grade;
						}
				}

			sfv_index++;
			PrintMyResultsToFile();
			}
	}

	if (sfv_index == 0)
		{
		std::cout << "\033[1;31m No SFV was successfully executed \033[1;31m" << std::endl;
		return(0);
		}

	for (int i=0 ; i<grads_sums.size() ; i++ )
	 {
		float Grade_mean = grads_sums[i]/sfv_index;							// E(x) = sum(x)/n
		float GradeSqer_mean = grads_sums_of_squers[i]/sfv_index;			// E(x^2) = sum(x^2)/n

		my_Grades_means->push_back(Grade_mean);
		my_Grades_stds->push_back(sqrt(GradeSqer_mean - Grade_mean*Grade_mean));    // Var(x) = E(x^2) - [E(x)]^2
	 }

	have_been_run = true;
    PrintMyResultsToFile();

	return 1;
}


TiXmlElement * SFDPobj::get_StatisticsInXML()
{
	if (! have_been_run)
	{
		std::cout << " \033[1;31m can't return Statistics In XML because the SFVs havn't been run yet \033[0m" << std::endl;
		return 0;
	}

	TiXmlElement * StatisticsXML = new TiXmlElement("Statistics");
	for (int i=0; i<my_Grades_means->size(); i++)
		{
		TiXmlElement * GradeXML = new TiXmlElement("Grade_"+std::to_string(i));

		GradeXML->SetAttribute("mean", std::to_string(my_Grades_means->at(i)));
		GradeXML->SetAttribute("std", std::to_string(my_Grades_stds->at(i)));
		StatisticsXML->LinkEndChild(GradeXML);
		}
	return(StatisticsXML);
}

TiXmlElement * SFDPobj::get_SFVsGradesInXML()
{
/*
	if (! have_been_run)
	{
		std::cout << " \033[1;31m can't return SFVs Grades In XML because the SFVs havn't been run yet \033[0m" << std::endl;
		return 0;
	}
*/
	TiXmlElement * SFV_GradesXML = new TiXmlElement("SFV_Grades");
	int sfv_index=0;
	std::string sfv_name;
	for (SFV * sfv_it : * my_sampled_SFVs )
		{
		if (sfv_it->get_WasExecutedFlag())
		  {	
			sfv_index = sfv_index + 1;
			sfv_name = "sfv_" + std::to_string(sfv_index);

			TiXmlElement * xml_sfv_grades = new TiXmlElement(sfv_name);
			xml_sfv_grades = sfv_it->get_GradesAsXMLElement(sfv_index);
			SFV_GradesXML->LinkEndChild(xml_sfv_grades);
		  }	
		}

	return(SFV_GradesXML);
}



TiXmlElement * SFDPobj::GetResultsInXML()
{
/*	
	if (! have_been_run)
	{
		std::cout << " \033[1;31m can't return results because the SFVs havn't been run yet \033[0m" << std::endl;
		return 0;
	}
*/

	TiXmlElement * resultsXML = new TiXmlElement("subSFDP_results");
	resultsXML->SetAttribute("dist_param_1", std::to_string(my_ExploretionFeature->get_dist_param_1()));
	resultsXML->SetAttribute("dist_param_2", std::to_string(my_ExploretionFeature->get_dist_param_2()));

	if (have_been_run)
	  {
	TiXmlElement * StatisticsXML = get_StatisticsInXML();
	resultsXML->LinkEndChild(StatisticsXML);
	  }

	TiXmlElement * SFV_GradesXML = get_SFVsGradesInXML();
	resultsXML->LinkEndChild(SFV_GradesXML);

	TiXmlElement * subSFDPsXML = new TiXmlElement("subSFDPs");
	resultsXML->LinkEndChild(subSFDPsXML);

	for (SFDPobj * subSFDP_it : * my_sub_SFDPs )
		{
			TiXmlElement * subSFDPresultsXML = subSFDP_it->GetResultsInXML();
			subSFDPsXML->LinkEndChild(subSFDPresultsXML);
		}

	return(resultsXML);
}

int SFDPobj::PrintMyResultsToFile()
{
	/*
	if (! have_been_run)
	{
		std::cout << " \033[1;31m can't print results file because the SFVs havn't been run yet \033[0m" << std::endl;
		return 0;
	}
	*/

	TiXmlDocument doc(my_Grades_file_url);
	TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
	doc.LinkEndChild(decl);

	TiXmlElement * xml_results = GetResultsInXML();
	doc.LinkEndChild(xml_results);

	doc.SaveFile(my_Grades_file_url.c_str());
	std::cout << " printing Grades to file : " << my_Grades_file_url << std::endl;

	return 1;
}



ScenarioFeature * SFDPobj::finedScenrioFeature(ScenarioFeatureGroupType GroupType, std::string GroupName, ScenarioFeatureType FeatureToLocate)
{
	for ( ScenarioFeatureGroup * group_it : * my_featureGroups )
		{
			if ( (group_it->get_featureGroupType() == GroupType) && (group_it->get_name() == GroupName) )
			{
				for (ScenarioFeature * feature_it : *(group_it->get_features()) )
				{
					if (feature_it->get_featureType() == FeatureToLocate)
					{
						return feature_it;
					}
				}
			}
		}
	std::cout << "\033[1;31m could not find GroupTipe " << GroupType << " = " << GroupName << " with feature " << FeatureToLocate << "\033[0m"<< std::endl;
	return 0;
}



int SFDPobj::SplitMe(ScenarioFeatureGroupType GroupTipe, std::string GroupName ,ScenarioFeatureType FeatureToSplit, float split_percents)
{
	if ( (! get_ExploretionFeature()))
		{
			std::cout << "\033[1;31m  could not split because feature to split wasn't found \033[0m" << std::endl;
			return 0;
		}

	SFDPobj * sub_sfdp1;
	SFDPobj * sub_sfdp2;

	std::string sub_sfdp_1_WS_url = my_WS_url + "sub_sfdp1/";
	std::string sub_sfdp_2_WS_url = my_WS_url + "sub_sfdp2/";

	boost::filesystem::remove_all(sub_sfdp_1_WS_url);
	boost::filesystem::remove_all(sub_sfdp_2_WS_url);

	boost::filesystem::create_directory(sub_sfdp_1_WS_url);
	boost::filesystem::create_directory(sub_sfdp_2_WS_url);


	if( ( boost::filesystem::create_directory(sub_sfdp_1_WS_url)) && ( boost::filesystem::create_directory(sub_sfdp_2_WS_url))  )
		{
		std::cout << "\033[1;31m failed to create folder for sub_sfdp_1_WS_url and/or sub_sfdp_2_WS_url at : " << std::endl;
		std::cout << sub_sfdp_1_WS_url << "\n" << sub_sfdp_2_WS_url << "\033[0m"<< std::endl;
		return 0;
		}


	sub_sfdp1 = new SFDPobj(sub_sfdp_1_WS_url+"sub_sfdp",my_Resources_file_url,sub_sfdp_1_WS_url,my_division_level+1);
	sub_sfdp1->set_FeatureGroups(this->get_FeatureGroups());
	sub_sfdp1->set_ExploretionFeature(sub_sfdp1->finedScenrioFeature(GroupTipe,GroupName,FeatureToSplit));

	sub_sfdp2 = new SFDPobj(sub_sfdp_2_WS_url+"sub_sfdp",my_Resources_file_url,sub_sfdp_2_WS_url,my_division_level+1);
	sub_sfdp2->set_FeatureGroups(this->get_FeatureGroups());
	sub_sfdp2->set_ExploretionFeature(sub_sfdp2->finedScenrioFeature(GroupTipe,GroupName,FeatureToSplit));


	ScenarioFeature * feature_sourse = this->get_ExploretionFeature(); // this->finedScenrioFeature(GroupTipe,GroupName,FeatureToSplit);
	ScenarioFeature * feature_1 = sub_sfdp1->get_ExploretionFeature(); // sub_sfdp1->finedScenrioFeature(GroupTipe,GroupName,FeatureToSplit);
	ScenarioFeature * feature_2 = sub_sfdp2->get_ExploretionFeature(); // sub_sfdp2->finedScenrioFeature(GroupTipe,GroupName,FeatureToSplit);



	if ( feature_sourse->get_distType() == ScenarioFeatureDistributionType::uniform_continuous )
		{
			float range = feature_sourse->get_dist_param_2() - feature_sourse->get_dist_param_1();
			float new_bound = feature_sourse->get_dist_param_1() + split_percents * range;
			feature_1->set_dist_param_2(new_bound);
			feature_2->set_dist_param_1(new_bound);
		}

	if ( feature_sourse->get_distType() == ScenarioFeatureDistributionType::uniform_discrete )
		{
			float range = feature_sourse->get_dist_param_2() - feature_sourse->get_dist_param_1();
			float new_bound = (int) (feature_sourse->get_dist_param_1() + split_percents * range);
			feature_1->set_dist_param_2(new_bound);
			feature_2->set_dist_param_1(new_bound);
		}

	if ( feature_sourse->get_distType() == ScenarioFeatureDistributionType::normal_continuous )
		{
			float mu = feature_sourse->get_dist_param_1();
			float sigma = feature_sourse->get_dist_param_2();

			feature_1->set_dist_param_1(mu-sigma);
			feature_1->set_dist_param_2(sigma/2);

			feature_2->set_dist_param_1(mu+sigma);
			feature_2->set_dist_param_2(sigma/2);
		}

	my_sub_SFDPs->push_back(sub_sfdp1);
	my_sub_SFDPs->push_back(sub_sfdp2);

	sub_sfdp1->PrintMeToFile();
	sub_sfdp2->PrintMeToFile();

	return 1;
}


int SFDPobj::ExploreMe(int argc, char** argv, int division_limit, int samples_number)
{
    this->set_DivisionLimit(division_limit);
	this->set_ExploretionFeature(this->finedScenrioFeature(ScenarioFeatureGroupType::obstacles_on_path,"obstacles_on_path" ,ScenarioFeatureType::number_of_obstacles_on_path));

	if ( (! get_ExploretionFeature()))
		{
			std::cout << "\033[1;31m  could not Explore because feature to explore wasn't found \033[0m" << std::endl;
			return 0;
		}

	if ( my_division_level > get_DivisionLimit() )
	{
		std::cout << "the division level reached the Division Limit " << std::endl;
		return 0;
	}

	if (! GenMySFVs(samples_number) )
	{
		std::cout << "\033[1;31m the generation of SFVs have failed \033[0m" << std::endl;
		return 0;
	}

	RunMySFVs(argc,argv);


	if (my_division_level < get_DivisionLimit())
	{
		if ((my_Grades_stds->at(0) > 0.1) || (samples_number==0) )
			{
				SplitMe(ScenarioFeatureGroupType::obstacles_on_path, "obstacles_on_path" ,ScenarioFeatureType::number_of_obstacles_on_path, 0.5);
				for (SFDPobj * sub_SFDP_it : * my_sub_SFDPs)
					{
						sub_SFDP_it->ExploreMe(argc,argv,get_DivisionLimit(),samples_number);
					}
			}
	}

	PrintMyResultsToFile();

	return 1;
}


