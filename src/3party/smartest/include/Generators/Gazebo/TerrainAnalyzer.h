/*
 * TerrainAnalizer.h
 *
 *  Created on: Feb 27, 2014
 *      Author: userws1
 */

#ifndef TERRAINANALYZER_H_
#define TERRAINANALYZER_H_
#include <opencv2/core/core.hpp>

class TerrainAnalyzer {
	float m_length;
	float m_width;
	float m_height;

	float m_lengthFactor;
	float m_widthFactor;
	float m_heightFactor;
	cv::Mat	m_heightmap;


	void parseDataFile(std::string path);
public:
	void loadFile(std::string path);
	TerrainAnalyzer();

	void getXYZCoord(float xPrecentage,float yPrecentage,float& xPos,float& yPos,float& zPos);
	void getZCoord(float xMeters,float yMeters,float& zPos);


	virtual ~TerrainAnalyzer();
	float getHeight() const;
	void setHeight(float height);
	float getLength() const;
	void setLength(float length);
	float getWidth() const;
	void setWidth(float width);
};

#endif /* TERRAINANALYZER_H_ */
