/*
 * sfvSubGroup.h
 *
 *  Created on: Jul 28, 2014
 *      Author: userws3
 */

#ifndef SFVSUBGROUP_H_
#define SFVSUBGROUP_H_

#include "SFDP/ScenarioFeatureGroup.h"
#include "SFDP/ScenarioFeature.h"
#include "SFDP/ScenarioFeatureType.h"
#include <vector>
#include <string>
#include <tinyxml.h>
#include "SFV/SFV.h"

#include <map>

class SFV;
class sfvSubGroup
{
	private :
		ScenarioFeatureGroupType my_type;
		std::string my_name;

		SFV * my_parent_SFV;

		std::map<ScenarioFeatureType,ScenarioFeature **> * my_features_map;
		bool was_rolled_flag;
	public:

		inline sfvSubGroup(ScenarioFeatureGroupType type, SFV * parent_SFV)
			{
			my_type = type;
			my_name = "";
			my_parent_SFV = parent_SFV;
			my_features_map = new std::map<ScenarioFeatureType ,ScenarioFeature** >;
			was_rolled_flag = false;
			}

		inline ScenarioFeatureGroupType get_Type()
			{return(my_type);}

		inline std::string get_Name()
			{ return(my_name); }

		inline void set_Name(std::string name)
			{ my_name = name; }

		inline SFV * get_ParentSFV()
			{return (my_parent_SFV); }

		inline bool get_WasRolledFlag()
			{ return(was_rolled_flag); }

		inline void set_WasRolledFlag(bool flag)
			{ was_rolled_flag = flag;}

		inline std::map<ScenarioFeatureType,ScenarioFeature **> * get_FeaturesMap()
				{return my_features_map; }


		inline virtual ~sfvSubGroup() {};

		inline virtual void initFeaturesMap() {};
		inline virtual bool  roll() {return(false);};
		inline virtual TiXmlElement * ToXmlElement(int id) {return(0);};

		void initSubGroupFeatures(std::vector<ScenarioFeature *> * ScenarioFeatures_vec);
		void cloneSubGroupFeatures(sfvSubGroup * template_group);
		void setSubGroupFeaturesFromXmlElement(TiXmlNode * xml_subGroup);

		void rollSubGroupfeatures();
		void resetSubGroupfeatures();
		TiXmlElement * SubGroupfeaturesToXmlElement(int id);

};

#endif /* SFVSUBGROUP_H_ */
