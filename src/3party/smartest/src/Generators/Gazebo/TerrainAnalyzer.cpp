/*
 * TerrainAnalyzer.cpp
 *
 *  Created on: Feb 27, 2014
 *      Author: userws1
 */

#include "Generators/Gazebo/TerrainAnalyzer.h"

#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <utils/TinyXmlDef.h>

void TerrainAnalyzer::parseDataFile(std::string path)
{
	TiXmlDocument doc(path+"/data.xml");
	if (!doc.LoadFile())
	{
		std::string error("Failed to load file \"");
		error+=path+"/data.xml";
		error+="\"\n";
		throw error;
	}

	TiXmlNode* pChild;
	//search for an sdfp element to parse
	for ( pChild = doc.FirstChild(); pChild != 0; pChild = pChild->NextSibling())
	{
		if(pChild->Type()==XML_ELEMENT && pChild->ValueStr().compare("data")==0){
			TiXmlNode* sChild;
			for (sChild = pChild->FirstChild(); sChild != 0; sChild = sChild->NextSibling())
			{
				if(sChild->Type()==XML_ELEMENT && sChild->ValueStr().compare("heightmap")==0)
				{
					std::string imageName=sChild->FirstChild()->ToText()->Value();
					m_heightmap=cv::imread(path+"/materials/textures/"+imageName);
				}
				if(sChild->Type()==XML_ELEMENT && sChild->ValueStr().compare("length")==0)
				{
					m_length=atof(sChild->FirstChild()->ToText()->Value());
				}
				if(sChild->Type()==XML_ELEMENT && sChild->ValueStr().compare("width")==0)
				{
					m_width=atof(sChild->FirstChild()->ToText()->Value());
				}
				if(sChild->Type()==XML_ELEMENT && sChild->ValueStr().compare("height")==0)
				{
					m_height=atof(sChild->FirstChild()->ToText()->Value());
				}
			}
			break;
		}
	}
}

TerrainAnalyzer::TerrainAnalyzer() {




}

void TerrainAnalyzer::loadFile(std::string filename)
{
	parseDataFile(filename);
	m_lengthFactor=m_length/m_heightmap.rows;
	m_widthFactor=m_width/m_heightmap.cols;
	m_heightFactor=m_height/255;
}

void TerrainAnalyzer::getXYZCoord(float xPrecentage,float yPrecentage,float& xPos,float& yPos,float& zPos)
{
	xPos=xPrecentage*m_width;
	yPos=yPrecentage*m_length;
	int i=xPos/m_widthFactor;
	int j=yPos/m_lengthFactor;
	zPos=m_heightmap.at<uchar>(j, i, 0)*m_heightFactor;
}

void TerrainAnalyzer::getZCoord(float xMeters,float yMeters,float& zPos)
{
	int i=xMeters/m_widthFactor;
	int j=yMeters/m_lengthFactor;
	zPos=m_heightmap.at<uchar>(j, i, 0)*m_heightFactor;
}


TerrainAnalyzer::~TerrainAnalyzer() {
	// TODO Auto-generated destructor stub
}

float TerrainAnalyzer::getHeight() const {
	return m_height;
}

void TerrainAnalyzer::setHeight(float height) {
	m_height = height;
}

float TerrainAnalyzer::getLength() const {
	return m_length;
}

void TerrainAnalyzer::setLength(float length) {
	m_length = length;
}

float TerrainAnalyzer::getWidth() const {
	return m_width;
}

void TerrainAnalyzer::setWidth(float width) {
	m_width = width;
}
