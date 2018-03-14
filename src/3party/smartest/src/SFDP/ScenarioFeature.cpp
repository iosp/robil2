/*
 * ScenarioFeature.cpp
 *
 *  Created on: Feb 3, 2014
 *      Author: userws1
 */

#include "SFDP/ScenarioFeature.h"
#include "utils/TinyXmlDef.h"

#include <string>
#include <stdlib.h>
#include <tinyxml.h>
#include <string>

ScenarioFeature::ScenarioFeature(std::string featureType):
	m_featureType(ScenarioFeatureType::parseString(featureType.c_str())),
	m_distType(ScenarioFeatureDistributionType::unknown_distribution),
	m_dist_param_1(0),
	m_dist_param_2(0)
{
	was_rolled_flag=false;
	rolled_value=0;
}


ScenarioFeature::ScenarioFeature():
	m_featureType(ScenarioFeatureType::unknown_feature),
	m_distType(ScenarioFeatureDistributionType::unknown_distribution),
	m_dist_param_1(0),
	m_dist_param_2(0)
{
	was_rolled_flag=false;
	rolled_value=0;
}

ScenarioFeature::ScenarioFeature(ScenarioFeature * source_ScenarioFeature)
{
	m_featureType=source_ScenarioFeature->get_featureType();
	m_distType=source_ScenarioFeature->get_distType();
	m_dist_param_1=source_ScenarioFeature->get_dist_param_1();
	m_dist_param_2=source_ScenarioFeature->get_dist_param_2();

	was_rolled_flag=false;
	rolled_value=0;
}



ScenarioFeature::ScenarioFeature(TiXmlNode* xml_ScenarioFeature):
	m_distType(ScenarioFeatureDistributionType::unknown_distribution),
	m_dist_param_1(0),
	m_dist_param_2(0)
{
	if (xml_ScenarioFeature->Type()==XML_ELEMENT)
		{
		m_featureType = ScenarioFeatureType::get_by_name(xml_ScenarioFeature->ValueStr().c_str()).get();
		rolled_value = std::stof(xml_ScenarioFeature->FirstChild()->ToText()->ValueStr());
		was_rolled_flag=true;
		}
}

void ScenarioFeature::roll()
{
	if(was_rolled_flag)
	{
		std::cout << "\033[1;31m I already was rolled (I am " <<  m_featureType << ")\033[0m"<< std::endl;
	}
	else
	{
		float result;
		switch(m_distType.index())
		{
			case ScenarioFeatureDistributionType::uniform_discrete:
				result= NumberSampler::getInstance().uniformDiscreteDistribution(m_dist_param_1,m_dist_param_2);
				break;
			case ScenarioFeatureDistributionType::uniform_continuous:
				result=NumberSampler::getInstance().uniformContinuousDistribution(m_dist_param_1,m_dist_param_2);
				break;
			case ScenarioFeatureDistributionType::normal_continuous:
				result=NumberSampler::getInstance().normalContinuousDistribution(m_dist_param_1,m_dist_param_2);
				break;
			case ScenarioFeatureDistributionType::unknown_distribution:
				throw std::string("unknown_distribution cannot be rolled");
		}
		rolled_value = result;
		was_rolled_flag=true;
	}
}



ScenarioFeature::~ScenarioFeature()
{
}


ScenarioFeatureType ScenarioFeature::ScenarioFeature::get_featureType()
{
	return m_featureType;
}

ScenarioFeatureDistributionType ScenarioFeature::get_distType()
{
	return m_distType;
}

float ScenarioFeature::get_dist_param_1()
{
	return m_dist_param_1;
}

float ScenarioFeature::get_dist_param_2()
{
	return m_dist_param_2;
}


void ScenarioFeature::set_dist_param_1(float val)
{
	m_dist_param_1=val;
}


void ScenarioFeature::set_dist_param_2(float val)
{
	m_dist_param_2=val;
}

void ScenarioFeature::set_featureType(ScenarioFeatureType type)
{
	m_featureType=type;
}


void ScenarioFeature::set_distributionType(ScenarioFeatureDistributionType type)
{
	m_distType=type;
}




int ScenarioFeature::parseScenarioFeatureFromXML(TiXmlNode* xmlFeature)
{
	std::string FeatureType = xmlFeature->ToElement()->FirstAttribute()->ValueStr();

	if ((xmlFeature->ValueStr().compare("scenario_feature")!=0) || (ScenarioFeatureType::get_by_name(FeatureType.c_str())==0))
	{
		std::cout << "\033[1;31m could not parse scenario_feature = " << FeatureType.c_str() << " because it is not a a valid Scenario Feature \033[0m" << std::endl;
		return 0;
	}

	std::string distribution = "";
	std::string dist_param_1 = "";
	std::string dist_param_2 = "";

	TiXmlNode* pChild;
	for ( pChild = xmlFeature->FirstChild(); pChild != 0; pChild = pChild->NextSibling())
	{
		if(pChild->Type()==XML_ELEMENT && pChild->ValueStr().compare("distribution")==0){
			distribution=pChild->FirstChild()->ToText()->ValueStr();
		}
		if(pChild->Type()==XML_ELEMENT && pChild->ValueStr().compare("dist_param_1")==0){
			dist_param_1=pChild->FirstChild()->ToText()->ValueStr();
		}
		if(pChild->Type()==XML_ELEMENT && pChild->ValueStr().compare("dist_param_2")==0){
			dist_param_2=pChild->FirstChild()->ToText()->ValueStr();
		}
	}

	if ( (distribution=="") || (dist_param_1=="") || (dist_param_2=="") || (ScenarioFeatureDistributionType::get_by_name(distribution.c_str())==0) )
	{
		std::cout << "\033[1;31m could not parse scenario_feature = " << FeatureType.c_str() << " its distribution, or distribution parameters are not valid " << std::endl;
		return 0;
	}

		m_featureType = ScenarioFeatureType::parseString(FeatureType.c_str());
		m_distType = ScenarioFeatureDistributionType::parseString(distribution.c_str());
		m_dist_param_1 = atof(dist_param_1.c_str());
		m_dist_param_2 = atof(dist_param_2.c_str());

	return 1;
}



TiXmlElement *ScenarioFeature::toXMLElement()
{
	TiXmlElement * xml_feature= new TiXmlElement("scenario_feature");
	xml_feature->SetAttribute("type",m_featureType.str());

		TiXmlElement * xml_dist= new TiXmlElement("distribution");
		TiXmlText * dist_val= new TiXmlText( m_distType.str() );
		xml_dist->LinkEndChild(dist_val);
		xml_feature->LinkEndChild(xml_dist);


		std::stringstream temp_ss;
		temp_ss.str("");
		temp_ss << m_dist_param_1;
		TiXmlElement * xml_dp1= new TiXmlElement("dist_param_1");
		TiXmlText * dp1_val= new TiXmlText( temp_ss.str() );
		xml_dp1->LinkEndChild(dp1_val);
		xml_feature->LinkEndChild(xml_dp1);

		temp_ss.str("");
		temp_ss << m_dist_param_2;
		TiXmlElement * xml_dp2= new TiXmlElement("dist_param_2");
		TiXmlText * dp2_val= new TiXmlText( temp_ss.str() );
		xml_dp2->LinkEndChild(dp2_val);
		xml_feature->LinkEndChild(xml_dp2);

	return(xml_feature);
}

TiXmlElement *ScenarioFeature::toSFV_XMLElement()
{
	if (! was_rolled_flag)
	{
		std::cout << "\033[1;31m can not make XML element for " << m_featureType.str() << " because it wasn't rolled yet \033[0m"<< std::endl;
		return 0;
	}
	else
	{
		TiXmlElement * xml_feature= new TiXmlElement(m_featureType.str());
		std::stringstream temp_ss;
		temp_ss.str("");
		temp_ss << rolled_value;
		TiXmlText * xml_rolled_val= new TiXmlText( temp_ss.str() );
		xml_feature->LinkEndChild(xml_rolled_val);
		return(xml_feature);
	}
}

