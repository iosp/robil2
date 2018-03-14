/*
 * SFDPobj.h
 *
 *  Created on: Jul 1, 2014
 *      Author: userws3
 */

#ifndef SFDPOBJ_H_
#define SFDPOBJ_H_

#include <string>
#include <vector>
#include <map>
#include <tinyxml.h>

#include "SFDP/ScenarioFeatureGroup.h"
#include "SFDP/ScenarioFeatureType.h"
#include "ScenarioFeatureDistributionType.h"

#include "SFV/SFV.h"

class SFV;

class SFDPobj {
	private :
		std::vector<ScenarioFeatureGroup*> * my_featureGroups; // stores the SFDP

		std::string my_SFDP_file_url;
		std::string my_Resources_file_url;
		std::string my_WS_url;     					  // all output files will be created in this folder
		std::string my_Grades_file_url;

		std::vector<SFV *> * my_sampled_SFVs;   // vector of sampled SFVs

		bool have_been_run;								// Flag to show that the Grades have been calculated
		std::vector<float> * my_Grades_means;			// Grade refers to the my_sampled_SFVs
		std::vector<float> * my_Grades_stds;

		std::vector<SFDPobj *> * my_sub_SFDPs;			// vector of sub SFDPs
		ScenarioFeature * my_ExploretionFeature;		// the feature that is explored (the split performed on it)
		int my_DivisionLimit;							// the limit of root SFDP devisions in exploration process
		int my_division_level;

	public :

		SFDPobj(std::string SFDP_file_url, std::string Resources_file_url, std::string WS_url, int division_level);
		~SFDPobj();

		int ExploreMe(int argc, char** argv, int division_limit, int samples_number);

		int ParseMeFromXMLFile();
		int PrintMeToFile();

		TiXmlElement * get_StatisticsInXML();
		TiXmlElement * get_SFVsGradesInXML();
		TiXmlElement * GetResultsInXML();
		int PrintMyResultsToFile();

//		SFDPobj * ShrinkMe(ScenarioFeatureType * FeatureToShrink, float new_upper_bound_percents, float new_lower_bound_percents);
		int SplitMe(ScenarioFeatureGroupType GroupTipe, std::string GroupName ,ScenarioFeatureType FeatureToSplit, float split_percents);


//		SFV* genSFVComp();
		int GenMySFVs(int samp_num);
		int RunMySFVs(int argc, char** argv);

		inline std::string get_SFDP_file_url()
			{ return my_SFDP_file_url; }

		inline std::string get_Resources_file_url()
			{ return my_Resources_file_url; }

		inline std::string get_WS_url()
			{ return my_WS_url; }

		inline int set_ExploretionFeature(ScenarioFeature * exploretion_feature)
			{ my_ExploretionFeature = exploretion_feature; return 1;}

		inline ScenarioFeature * get_ExploretionFeature()
			{ return my_ExploretionFeature; }

		inline int get_division_level()
			{ return my_division_level; }

		inline void set_DivisionLimit(int DivisionLimit)
		    { my_DivisionLimit = DivisionLimit; }

		inline int get_DivisionLimit()
		    { return(my_DivisionLimit); }

		inline std::vector<float> * get_Garades_means()
			{ if (have_been_run) return my_Grades_means; else return 0; }

		inline std::vector<float> * get_Garades_stds()
			{ if (have_been_run) return my_Grades_stds; else return 0; }

		inline std::vector<ScenarioFeatureGroup*> * get_FeatureGroups()
			{if (! my_featureGroups->empty() ) return my_featureGroups; else return 0; }


		ScenarioFeature * finedScenrioFeature(ScenarioFeatureGroupType GroupType, std::string GroupName, ScenarioFeatureType FeatureToLocate);


		inline int set_FeatureGroups(std::vector<ScenarioFeatureGroup*> * source_featureGroups)
			{
			my_featureGroups = new std::vector<ScenarioFeatureGroup*>;
			ScenarioFeatureGroup * temp_group;
			for ( ScenarioFeatureGroup * group_it : * source_featureGroups)
				{
					temp_group = new ScenarioFeatureGroup(group_it);
					my_featureGroups->push_back(temp_group);
				}
			return 1;
			}


		inline std::vector<SFV *> * get_Sampled_SFVs()
			{if (! my_sampled_SFVs->empty() ) return my_sampled_SFVs; else return 0; }

		inline std::vector<SFDPobj *> * get_Sub_SFDPs()
			{if (! my_sub_SFDPs->empty() ) return my_sub_SFDPs; else return 0; }
};


#endif /* SFDPOBJ_H_ */
