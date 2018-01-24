/*
 * SFV.h
 *
 *  Created on: Jul 30, 2014
 *      Author: userws3
 */

#ifndef SFV_H_
#define SFV_H_

#include <iostream>
#include <vector>
#include <string>

#include "SFDP/ScenarioFeatureGroupType.h"
#include "SFV/sfvSubGroup.h"

#include "SFDP/SFDPobj.h"

#include "Rules/Rule.h"

#include "std_msgs/Float32MultiArray.h"


class SFDPobj;
class sfvSubGroup;

class SFV {
	private :
		std::vector<sfvSubGroup *> * my_sfvSubGroups;

		std::string my_resource_file_url;
		std::string my_ws_folder_url;

		SFDPobj * my_SFDP;
		bool was_rolled_flag;

		std::vector<Rule *> * my_rules;

		bool was_executed_flag;
		std_msgs::Float32MultiArray::ConstPtr  my_grades;

		std::vector<sfvSubGroup*> * SubGroupsBayFeatureGroupType_ReturnVec;

	public:
		SFV(SFDPobj * SFDP, std::string ws_folder_url);
		SFV(std::string SFV_file_name, std::string ws_folder_url);

		bool roll();
		bool rules_check();

		int printToXML(std::string sfv_file_url);

		int generate();
		int execute(int argc, char** argv);

		TiXmlElement *get_GradesAsXMLElement(int sfv_idex);
		int PrintResultsToFile();

		~SFV();

		inline std::vector<sfvSubGroup *> * get_sfvSubGroups()
			{ if (! my_sfvSubGroups->empty() ) { return my_sfvSubGroups; }
				  else { std::cout << "\033[1;31m SFV SubGroups vector is empty \033[0m" << std::endl; return(0); } }

		inline std::string get_WSfolder()
			{ return(my_ws_folder_url); }

		inline std::string get_ResourceFile()
			{ return(my_resource_file_url); }

		inline SFDPobj * get_SFDP()
			{
			if (my_SFDP == 0) { std::cout << " the is no SFDP perent defined for this SFV, it probably was created from file " << std::endl; return(0); }
			else { return(my_SFDP); }
			}

		sfvSubGroup * get_SubGroupByFeatureGroupType(ScenarioFeatureGroupType GroupType);
		bool get_VecOfSubGroupsByFeatureGroupType(ScenarioFeatureGroupType GroupType, std::vector<sfvSubGroup *> * SubGroupVec);

		template <class T>
		void Populate_mySFVsubGroups(ScenarioFeatureGroupType::optional scenarioFeatureType , T fatures_data);

		inline bool get_WasRolledFlag()
			{ return(was_rolled_flag); }


		inline bool get_WasExecutedFlag()
			{ return(was_executed_flag); }

		inline std_msgs::Float32MultiArray::ConstPtr get_Grades()
			{ if (was_executed_flag) { return my_grades; }
				else { std::cout << "\033[1;31m SFV wasn't executed yet \033[0m" << std::endl; return(0); } }

};





#endif /* SFV_H_ */
