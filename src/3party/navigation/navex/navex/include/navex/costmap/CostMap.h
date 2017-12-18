/*
 * Filename: CostMap.h
 *   Author: Igor Makhtes
 *     Date: Nov 26, 2014
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#ifndef INCLUDE_NAVEX_COSTMAP_H_
#define INCLUDE_NAVEX_COSTMAP_H_


#include <string>

#include <boost/noncopyable.hpp>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>

#include <navex/costmap/CostMapCell.h>
#include <navex/costmap/datasource/CostMapDataSourceBase.h>
#include <navex/costmap/CostMapDataContainer.h>
#include <navex/costmap/parameters/IParametersProvider.h>


using namespace std;


/**
 * Generic cost map representation.
 * Can be used with various data sources
 * @see IDataSource
 */
class CostMap : public boost::noncopyable {

public:

	/**
	 * Initializes the cost map with specified parameters
	 * @param dataSource Data source for cost map filling
	 * @param robotRadius Robot's radius in meters
	 * @param inflationRadius Inflation radius in meters
	 * @param mapWidth Map width in meters
	 * @param mapHeight Map height in meters
	 * @param resolution
	 * @param frameId Robot's frame id, all points from the ICostMapDataSource will be transformed into this frame.
	 * 				  It also will be the frame id of the published map
	 * @warning Currently the origin of the map it the center
	 * @todo add originX, originY
	 */
	CostMap(CostMapDataSourceBase* dataSource, double robotRadius, double inflationRadius, double inflation_siggma, double inflation_pow,
			double mapWidth, double mapHeight, double resolution, const string& frameId);

	/**
	 * Initializes the cost map with specified parameters
	 * @param dataSource
	 * @param parametersProvider
	 * @param createMap If true, the map array will be allocated using parameters provided
	 *                  by parametersProvider, otherwise, map array will not be allocated,
	 *                  meaning the dataSource is responsible for map creation,
	 *                  @note in current implementation in this case, only inflation radius
	 *                  parameter will be used
	 */
	CostMap(CostMapDataSourceBase* dataSource, IParametersProvider* parametersProvider,
			bool createMap = true);

	/**
	 * Initializes empty cost map, assumes dataSource will fill it later
	 * @param dataSource
	 * @param inflationRadius
	 */
	CostMap(CostMapDataSourceBase* dataSource, double robotRadius,
			double inflationRadius, double inflation_siggma, double inflation_pow
			);

	/**
	 * Deletes data source and parameters provider pointers if needed
	 */
	virtual ~CostMap();

public:

	/**
	 * Returns occupancy grid
	 * @return occupancy grid
	 */
	inline nav_msgs::OccupancyGrid::ConstPtr getOccupancyGrid() const {
		return boost::const_pointer_cast<nav_msgs::OccupancyGrid const>(occupancyGrid_);
	}

	/**
	 * Returns cv::Mat of the cost map
	 * @return
	 */
	inline const cv::Mat& getCvMatrix() const {
		return cvMatrix_;
	}

	/**
	 * Gets map width in pixels
	 * @return
	 */
	inline uint32_t getWidth() const {
		return cvMatrix_.cols;
	}

	/**
	 * Gets map height in pixels
	 * @return
	 */
	inline uint32_t getHeight() const {
		return cvMatrix_.rows;
	}

	/**
	 * Gets map's frame id
	 * @return
	 */
	inline string getFrameId() const {
		return occupancyGrid_->header.frame_id;
	}



	/**
	 * Gets inflation radius in meters
	 * @return
	 */
	inline double getInflation() const {
		return inflationRadius_;
	}

	/**
	 * Gets robot's radius in meters
	 * @return
	 */
	inline double getRobotRadius() const {
		return robotRadius_;
	}

	/**
	 * Set all cells of the map to free(0)
	 */
	inline void clearMap() {
		boost::recursive_mutex::scoped_lock lock(readWriteMutex_);

		memset(occupancyGrid_->data.data(), 0, occupancyGrid_->data.size());
	}

	/**
	 * Returns the value of cost map cell in specified cell
	 * @param x X cell index
	 * @param y Y cell index
	 * @return
	 */
	CostMapCell::CellType getCellValue(uint32_t x, uint32_t y) const;

	/**
	 * Returns the value of cost map cell, located in specified position
	 * @param pose Position should be in cost map's frame
	 * @return Cell value: Unknown(-1), Free(0), Occupied(1..100)
	 */
	CostMapCell::CellType getCellValue(const geometry_msgs::Pose& pose) const;

	/**
	 * Returns the value of cost map cell, located in specified position with specified frame
	 * @param poseStamped
	 * @return
	 */
	CostMapCell::CellType getCellValue(const geometry_msgs::PoseStamped& poseStamped) const;

	/**
	 * Returns the value of cost map cell, located in specified position with specified frame
	 * @param x
	 * @param y
	 * @param frameId
	 * @param stamp
	 * @return
	 */
	CostMapCell::CellType getCellValue(double x, double y,
			const string& frameId, const ros::Time& stamp = ros::Time(0)) const;

	/**
	 * Sets a value to specified cell
	 * @param pixel
	 * @param value
	 * @return True if change was made, false otherwise
	 */
	inline bool setCellValue(cv::Point pixel, CostMapCell::CellType value) {
		boost::recursive_mutex::scoped_lock lock(readWriteMutex_);

		if (!isInBounds(pixel))
			return false;

		cvMatrix_.at<CostMapCell::CellType>(pixel) = value;

		return true;
	}

	/**
	 * Converts pixel coordinates to stamped pose in map's frame
	 * @param x
	 * @param y
	 * @return
	 */
	geometry_msgs::PoseStamped pixelToPose(uint32_t x, uint32_t y) const;
	geometry_msgs::PoseStamped pixelToPose(int x, int y) const{return pixelToPose((uint32_t)x,(uint32_t)y);}
	geometry_msgs::PoseStamped pixelToPose(long x, long y) const{return pixelToPose((uint32_t)x,(uint32_t)y);}
	geometry_msgs::PoseStamped pixelToPose(double x, double y) const;

	/**
	 * Converts pixel coordinates to stamped pose in map's frame
	 * @param pixel
	 * @return
	 */
	geometry_msgs::PoseStamped pixelToPose(cv::Point pixel) const;
	geometry_msgs::PoseStamped pixelToPose(cv::Point2d pixel) const;

	/**
	 * Converts pose in arbitrary frame to pixel coordinates
	 * @param pose
	 * @return
	 */
	cv::Point poseToPixel(const geometry_msgs::PoseStamped& pose) const;

	/**
	 * Convert local map coordinates to pixels coordinates
	 * @param point
	 * @return
	 */
	inline cv::Point localCoordinatesToPixel(const tf::Vector3& point) const {
		return localCoordinatesToPixel(point.x(), point.y());
	}

	/**
	 * Convert local map coordinates to pixels coordinates
	 * @param x
	 * @param y
	 * @return
	 */
	inline cv::Point localCoordinatesToPixel(double x, double y) const {
		return cv::Point(
				(x - occupancyGrid_->info.origin.position.x) /
					occupancyGrid_->info.resolution,
				(y - occupancyGrid_->info.origin.position.y) /
					occupancyGrid_->info.resolution);
	}

	/**
	 * Checks whether provided pixel within the map bounds
	 * @param pixel
	 * @return If the point within the map bounds - the value returned,
	 * 		   otherwise Unknown(-1)
	 */
	inline bool isInBounds(const cv::Point& pixel) const {
		return pixel.x >= 0 && pixel.x < cvMatrix_.cols &&
				pixel.y >= 0 && pixel.y < cvMatrix_.rows;
	}

	/**
	 * Sets specified cell (or region) to free
	 * @param pixel
	 * @param radius Radius in meters
	 */
	void clearCell(const cv::Point& pixel, double radius);

	/**
	 * Sets specified cell (or region) to free
	 * @param pose
	 * @param radius Radius in meters
	 */
	inline void clearCell(const geometry_msgs::PoseStamped& pose, double radius) {
		clearCell(poseToPixel(pose), radius);
	}

	/**
	 * Merges contents of provided costmap with current map
	 * @param ignoreFree If true, only obstacles are merged (cell >= CELL_BLOCKED)
	 * @param costmap
	 */
	void merge(const CostMap& costmap, bool ignoreFree = true);

protected:

	/**
	 * Data source for the cost map
	 */
	CostMapDataSourceBase* dataSource_;

	/**
	 * Cost map parameters provider
	 */
	IParametersProvider* parametersProvider_;

	/**
	 * Holds the grid array
	 */
	nav_msgs::OccupancyGrid::Ptr occupancyGrid_;

	/**
	 * Uses the array of occupancy grid
	 */
	cv::Mat cvMatrix_;

	/**
	 * Transform listener
	 */
	tf::TransformListener tfListener_;

	/**
	 * Robot's radius
	 */
	double robotRadius_;

	/**
	 * Inflation radius in meters
	 */
	double inflationRadius_;
	double inflation_template__inflation_sigmma_, inflation_template__inflation_pow_;

	/**
	 * Raw points data
	 */
	CostMapDataContainer points_;

	/**
	 * Costmap read/write mutex
	 */
	mutable boost::recursive_mutex readWriteMutex_;

protected:

	/**
	 * Initializes the cost map with specified parameters
	 * @param dataSource Data source for cost map filling
	 * @param parametersProvider Cost map parameters provider
	 * @param inflationRadius Inflation radius in meters
	 * @param mapWidth Map width in meters
	 * @param mapHeight Map height in meters
	 * @param resolution
	 * @param frameId Robot's frame id, all points from the ICostMapDataSource will be transformed into this frame.
	 * 				  It also will be the frame id of the published map
	 * @param allocateMapArray If true, an array will be allocated for the map
	 * @warning Currently the origin of the map is the center
	 * @todo add originX, originY
	 */
	void initializeCostMap(CostMapDataSourceBase* dataSource,
			IParametersProvider* parametersProvider, double robotRadius,
			double inflationRadius, double inflation_siggma, double inflation_pow,
			double mapWidth,double mapHeight, double resolution,
			string frameId,
			bool allocateMapArray = true);

	/**
	 * Prints summary to ROS console
	 */
	void printSummary() const;

	/**
	 * Creates the occupancy grid message
	 * @param mapWidth Map width in meters
	 * @param mapHeight Map height in meters
	 * @param originX Map's x origin in meters
	 * @param originY Map's y origin in meters
	 * @param resolution
	 * @param frameId
	 */
	void createOccupancyGrid(double mapWidth,
			double mapHeight, double originX, double originY,
			double resolution, const string& frameId);

	/**
	 * Clear cost map callback from ICostMapDataSource
	 */
	inline void clearMapCallback() {
		clearMap();
	}

	/**
	 * Updates map data with new provided data
	 * @param dataContainer
	 */
	void updatePoints(const CostMapDataContainer& dataContainer);

	/**
	 * Updates parameters using @see IParametersProvider
	 */
	void updateParameters();


	cv::Mat& get_template_for_inflation(double maybeBlockedInflation, double blockedInflation);
	cv::Mat create_template_for_inflation(double maybeBlockedInflation, double blockedInflation);
	void apply_inflation(const cv::Mat& inflation_template_, cv::Mat& cvMatrix_);
	void apply_inflation(const cv::Mat& inflation_template_, cv::Mat& cvMatrix_, const cv::Point& pixel);


};

#endif /* INCLUDE_NAVEX_COSTMAP_H_ */
