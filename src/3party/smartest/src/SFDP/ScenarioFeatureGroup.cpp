/*
 * ScenarioFeatureGroup.cpp
 *
 *  Created on: Feb 3, 2014
 *      Author: userws1
 */

#include "SFDP/ScenarioFeatureGroup.h"
#include "utils/TinyXmlDef.h"

#include <string>
#include <stdlib.h>
#include <tinyxml.h>


ScenarioFeatureGroup::ScenarioFeatureGroup() :
m_featureGroupType(ScenarioFeatureGroupType::unknown_feature_group)
{
	m_name="";
	m_features=(new std::vector<ScenarioFeature *>);
}

ScenarioFeatureGroup::ScenarioFeatureGroup(std::string featureType):
	m_featureGroupType(ScenarioFeatureGroupType::parseString(featureType.c_str()))
{
	m_features=(new std::vector<ScenarioFeature *>);
	m_name="";
	set_featureGroupType(ScenarioFeatureGroupType::parseString(featureType.c_str()));
}

ScenarioFeatureGroup::ScenarioFeatureGroup(ScenarioFeatureGroup * source_ScenarioFeatureGroup)
{
	m_name = source_ScenarioFeatureGroup->m_name;
	m_featureGroupType = source_ScenarioFeatureGroup->m_featureGroupType;
	m_features=(new std::vector<ScenarioFeature *>);

	ScenarioFeature * feature_temp;
	for (ScenarioFeature * feature_it : *(source_ScenarioFeatureGroup->get_features()) )
	{
		feature_temp = new ScenarioFeature(feature_it);
		m_features->push_back(feature_temp);
	}
}

ScenarioFeatureGroup::~ScenarioFeatureGroup()
{
	delete m_features;
}


ScenarioFeatureGroupType ScenarioFeatureGroup::get_featureGroupType()
{
	return m_featureGroupType;
}
void ScenarioFeatureGroup::set_featureGroupType(ScenarioFeatureGroupType type)
{
	m_featureGroupType=type;
}

std::string ScenarioFeatureGroup::get_name()
{
	return m_name;
}

void ScenarioFeatureGroup::set_name(std::string name)
{
	m_name=name;
}

std::vector<ScenarioFeature*> * ScenarioFeatureGroup::get_features()
{
	return m_features;
}

void ScenarioFeatureGroup::addFeature(ScenarioFeature* feature)
{
	m_features->push_back(feature);
}



TiXmlElement *ScenarioFeatureGroup::toXMLElement()
{
	TiXmlElement * xml_featureGroup= new TiXmlElement("scenario_feature_group");
	xml_featureGroup->SetAttribute("type",m_featureGroupType.str());
	xml_featureGroup->SetAttribute("name",m_name);
	for (ScenarioFeature * feat_it : * m_features )
	{
		xml_featureGroup->LinkEndChild(feat_it->toXMLElement());
	}
	return(xml_featureGroup);
}


int ScenarioFeatureGroup::parseScenarioFeatureGroupFromXML(TiXmlNode* xmlFeatureGroup)
{
	if ( xmlFeatureGroup->ValueStr().compare("scenario_feature_group")!=0)
	{
		std::cout <<  "\033[1;31m could not parse " << xmlFeatureGroup->ValueStr().c_str() << " because it is not a Scenario Feature Group \033[0m" << std::endl;
		return 0;
	}

	std::string GroupType = "";
	std::string GroupName = "";

	TiXmlAttribute* pAttrib;
	for ( pAttrib = xmlFeatureGroup->ToElement()->FirstAttribute(); pAttrib != 0; pAttrib = pAttrib->Next())
		{
		if(pAttrib->NameTStr().compare("type")==0)
			{
			GroupType = pAttrib->ValueStr();
			}
		if(pAttrib->NameTStr().compare("name")==0)
			{
			GroupName = pAttrib->ValueStr();
			}
		}

	if ( (GroupType=="" ) || (GroupName=="") ||  (ScenarioFeatureGroupType::get_by_name(GroupType.c_str())==0) )
	{
		std::cout <<  "\033[1;31m could not parse scenario_feature_group = " << GroupType.c_str() << " its type, or name are not valid \033[0m" << std::endl;
		return 0;
	}

	m_name = GroupName;
	m_featureGroupType = ScenarioFeatureGroupType::parseString(GroupType.c_str());

	ScenarioFeature *feature;
	TiXmlNode* pChild;
	for ( pChild = xmlFeatureGroup->FirstChild(); pChild != 0; pChild = pChild->NextSibling())
		{
		if(pChild->Type()==XML_ELEMENT && pChild->ValueStr().compare("scenario_feature")!=0)
			{
			std::cout <<  "\033[1;31m could not parse scenario_feature_group = " << GroupType.c_str() << "  one ore more of its chide features is not valid \033[0m" << std::endl;
			return 0;
			}
		else
			{
			feature = new ScenarioFeature();
			if (feature->parseScenarioFeatureFromXML(pChild))
				{
				addFeature(feature);
				}
			else
				{
				std::cout <<  "\033[1;31m could not parse scenario_feature_group = " << GroupType.c_str() << "  one ore more of its chide features is not valid \033[0m" << std::endl;
				return 0;
				}
			}
		}


	return 1;
}

