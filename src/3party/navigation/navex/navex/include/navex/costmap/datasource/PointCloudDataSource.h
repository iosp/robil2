/*
 * Filename: PointCloudDataSource.h
 *   Author: Igor Makhtes
 *     Date: Jan 4, 2016
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016
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

#ifndef INCLUDE_NAVEX_COSTMAP_DATASOURCE_POINTCLOUDDATASOURCE_H_
#define INCLUDE_NAVEX_COSTMAP_DATASOURCE_POINTCLOUDDATASOURCE_H_


#include <ros/ros.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl_conversions/pcl_conversions.h>

#include <tf/tf.h>

#include <navex/costmap/datasource/CostMapDataSourceBase.h>


using namespace std;


/**
 * Uses @see sensor_msgs::PointCloud2 as data source for @see CostMap
 */
template <class PointT>
class PointCloudDataSource : public CostMapDataSourceBase {

public:

	PointCloudDataSource(const string& topic, const string& baseFrame,
			ros::NodeHandle nodeHandle = ros::NodeHandle(),
			double minHeight = -0.5, double maxHeight = 3.2)
		: baseFrame_(baseFrame), minHeight_(minHeight), maxHeight_(maxHeight) {

		cloudSubscriber_ = nodeHandle.subscribe(topic, 10, &PointCloudDataSource::pointCloudCallback, this);
	}

	virtual ~PointCloudDataSource() {

	}

public:

	virtual inline string getName() const {
		return "PointCloud";
	}

private:

	string baseFrame_;

	ros::Subscriber cloudSubscriber_;

	/**
	 * Points above this height (and below maximum height) considered obstacles
	 */
	double minHeight_;

	/**
	 * Points below this height (and below minimum height) considered obstacles
	 */
	double maxHeight_;

	tf::TransformListener tfListener_;

private:

	void pointCloudCallback(const pcl::PointCloud<PointT>& cloud) {
//		pcl::PointCloud<PointT> transformedCloud;
//		pcl_ros::transformPointCloud(baseFrame_, cloud, transformedCloud, tfListener_);

		CostMapDataSourceBase::clearMap();

		CostMapDataContainer points(false, baseFrame_, pcl_conversions::fromPCL(cloud.header).stamp);

		// 5-15 => 0.0-1.0
		double min = 5.0;
		double max = 15.0;
		double nmin = 0.0;
		double nmax = 1.0;
		for (int i = 0; i < cloud.points.size(); ++i) {
			const double& x = cloud.points[i].x;
			const double& y = cloud.points[i].y;
			const double& z = cloud.points[i].z;

			double d = sqrt(x*x + y*y + z*z);
			double f = fabs((nmax-nmin)/(max-min)*(d-max)+nmax);

//			ROS_INFO_STREAM(z << "," << f << "," << minHeight_*f);

			if (z >= minHeight_ && z <= maxHeight_)
				points.push_back(PointData(x, y, 0, CostMapCell::CELL_BLOCKED));
		}

		updatePoints(points);
	}

};

#endif /* INCLUDE_NAVEX_COSTMAP_DATASOURCE_POINTCLOUDDATASOURCE_H_ */
