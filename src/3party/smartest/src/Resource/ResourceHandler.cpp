/*
 * ResourceHandler.cpp
 *
 *  Created on: Feb 24, 2014
 *      Author: userws1
 */

#include "Resource/ResourceHandler.h"
#include "utils/TinyXmlDef.h"

void parseResource(TiXmlNode* node,std::map<std::string,std::map<int,std::string> *>* resourceMap)
{
	TiXmlNode* pChild;
	for ( pChild = node->FirstChild(); pChild != 0; pChild = pChild->NextSibling())
	{
		TiXmlAttribute* id=pChild->ToElement()->FirstAttribute();
		TiXmlText * text=pChild->FirstChild()->ToText();
		resourceMap->at(pChild->ValueStr())->insert(std::pair<int,std::string>(atoi(id->Value()),text->ValueStr()));
	}
}


ResourceHandler::ResourceHandler(std::string resources_file_path) {

	m_resourceMap=new std::map<std::string,std::map<int,std::string> *>;
	m_resourceMap->insert(std::pair<std::string,std::map<int,std::string> *>(std::string("terrain"),new std::map<int,std::string>));
	m_resourceMap->insert(std::pair<std::string,std::map<int,std::string> *>(std::string("object"),new std::map<int,std::string>));
	m_resourceMap->insert(std::pair<std::string,std::map<int,std::string> *>(std::string("light"),new std::map<int,std::string>));

	my_RobotSensorsNames = new std::vector<std::string *>;

	TiXmlDocument doc(resources_file_path);

		if (!doc.LoadFile())
		{
			std::string error("Failed to load file \"");
			error+="resources/resource.xml";
			error+="\"\n";
			throw error;
		}
		TiXmlNode* pChild;
		//search for an sdfp element to parse
		for ( pChild = doc.FirstChild(); pChild != 0; pChild = pChild->NextSibling())
		{
			if(pChild->Type()==XML_ELEMENT && pChild->ValueStr().compare("robot_PyInterface")==0)
						{
							for ( TiXmlAttribute *PyInterface_xmlAttribute = pChild->ToElement()->FirstAttribute(); PyInterface_xmlAttribute != 0; PyInterface_xmlAttribute = PyInterface_xmlAttribute->Next())
							{
								if(PyInterface_xmlAttribute->NameTStr().compare("name")==0)
								{
									my_Robot_PyInterface=PyInterface_xmlAttribute->ValueStr();
								}
							}
						}

			if(pChild->Type()==XML_ELEMENT && pChild->ValueStr().compare("world_components_models")==0)
			{
				for ( TiXmlAttribute * world_components_xmlAttribute = pChild->ToElement()->FirstAttribute(); world_components_xmlAttribute != 0; world_components_xmlAttribute = world_components_xmlAttribute->Next())
				{
					if(world_components_xmlAttribute->NameTStr().compare("dirPath")==0)
					{
						my_WorldModelsFolderURL=world_components_xmlAttribute->ValueStr();
					}
				}
				parseResource(pChild,m_resourceMap);
			}

			if(pChild->Type()==XML_ELEMENT && pChild->ValueStr().compare("robot_components_models")==0)
			{
				for ( TiXmlAttribute * robot_components_xmlAtribute = pChild->ToElement()->FirstAttribute(); robot_components_xmlAtribute != 0; robot_components_xmlAtribute = robot_components_xmlAtribute->Next())
				{
					if(robot_components_xmlAtribute->NameTStr().compare("name")==0)
					{
						my_RobotName = robot_components_xmlAtribute->ValueStr();
					}
					if(robot_components_xmlAtribute->NameTStr().compare("dirPath")==0)
					{
						my_RobotModelsFolderURL=robot_components_xmlAtribute->ValueStr();
					}
				}

				for ( TiXmlNode * robot_component_xmlNode = pChild->FirstChild(); robot_component_xmlNode != 0; robot_component_xmlNode=robot_component_xmlNode->NextSibling())
				{
					if( robot_component_xmlNode->Type()==XML_ELEMENT && robot_component_xmlNode->ValueStr().compare("platform")==0)
					{
						my_RobotPlatformName = robot_component_xmlNode->ToElement()->FirstAttribute()->Value();
					}
					if( robot_component_xmlNode->Type()==XML_ELEMENT && robot_component_xmlNode->ValueStr().compare("sensor")==0)
					{
						std::string * sensor_name = new std::string(robot_component_xmlNode->ToElement()->FirstAttribute()->Value());
						my_RobotSensorsNames->push_back(sensor_name);
					}
				}
			}


		}
}

ResourceHandler::~ResourceHandler() {

	for (std::map<std::string,std::map<int,std::string> *>::iterator it=m_resourceMap->begin();
																			it!=m_resourceMap->end();
																								++it)
	{
		delete it->second;
	}
	delete m_resourceMap;
}


std::string ResourceHandler::getResource(std::string name,int id)
{
	if (m_resourceMap->find(name) != m_resourceMap->end() )
	{
		if(m_resourceMap->at(name)->find(id) != (m_resourceMap->at(name))->end() )
		{
			return m_resourceMap->at(name)->at(id);
		}
	}
	return("");
}

std::string ResourceHandler::getTerrainById(int id) {
	return getResource(std::string("terrain"),id);
}

std::string ResourceHandler::getObjectById(int id) {
	return getResource(std::string("object"),id);
}

std::string ResourceHandler::getLightById(int id) {
	return getResource(std::string("light"),id);
}


